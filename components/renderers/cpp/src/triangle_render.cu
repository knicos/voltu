#include <ftl/render/render_params.hpp>
#include "splatter_cuda.hpp"
#include <ftl/rgbd/camera.hpp>
#include <ftl/cuda_common.hpp>
#include <ftl/cuda/fixed.hpp>

#include <ftl/cuda/weighting.hpp>

using ftl::rgbd::Camera;
using ftl::cuda::TextureObject;
using ftl::render::Parameters;

#define T_PER_BLOCK 8

__device__ inline float length2(int dx, int dy) { return dx*dx + dy*dy; }

__device__ inline float cross(const float2 &a, const float2 &b) {
	return a.x*b.y - a.y*b.x;
}

__device__ inline bool within(float x) {
    return 0.0f <= x <= 1.0f;
}

__device__ inline bool operator==(const float2 &a, const float2 &b) {
	return a.x == b.x && a.y == b.y;
}

__device__ inline bool insideTriangle(const float2 &a, const float2 &b, const float2 &c, const float2 &p)
{   
    float det = (b.y - c.y)*(a.x - c.x) + (c.x - b.x)*(a.y - c.y);
    float factor_alpha = (b.y - c.y)*(p.x - c.x) + (c.x - b.x)*(p.y - c.y);
    float factor_beta = (c.y - a.y)*(p.x - c.x) + (a.x - c.x)*(p.y - c.y);
    float alpha = factor_alpha / det;
    float beta = factor_beta / det;
    float gamma = 1.0 - alpha - beta;

    return p == a || p == b || p == c || (within(alpha) && within(beta) && within(gamma));
}

__device__ inline void swap(short2 &a, short2 &b) {
	short2 t = a;
	a = b;
	b = t;
}

__device__ void drawLine(TextureObject<int> &depth_out, int y, int x1, int x2, float d) {
	for (int x=x1; x<=x2; ++x) {
		if (x < 0) continue;
		if (x >= depth_out.width()) return;
		atomicMin(&depth_out(x,y), int(d*1000.0f));
	}
}

/* See: https://github.com/bcrusco/CUDA-Rasterizer */

/**
 * Calculate the signed area of a given triangle.
 */
__device__ static inline
 float calculateSignedArea(const short2 &a, const short2 &b, const short2 &c) {
	 return 0.5f * (float(c.x - a.x) * float(b.y - a.y) - float(b.x - a.x) * float(c.y - a.y));
 }

/**
 * Helper function for calculating barycentric coordinates.
 */
 __device__ static inline
 float calculateBarycentricCoordinateValue(const short2 &a, const short2 &b, const short2 &c, const short2 (&tri)[3]) {
	 return calculateSignedArea(a,b,c) / calculateSignedArea(tri[0], tri[1], tri[2]);
 }
 
 /**
  * Calculate barycentric coordinates.
  * TODO: Update to handle triangles coming in and not the array
  */
__device__ static
 float3 calculateBarycentricCoordinate(const short2 (&tri)[3], const short2 &point) {
	 float beta = calculateBarycentricCoordinateValue(tri[0], point, tri[2], tri);
	 float gamma = calculateBarycentricCoordinateValue(tri[0], tri[1], point, tri);
	 float alpha = 1.0f - beta - gamma;
	 return make_float3(alpha, beta, gamma);
 }
 
 /**
  * Check if a barycentric coordinate is within the boundaries of a triangle.
  */
 __host__ __device__ static
 bool isBarycentricCoordInBounds(const float3 &barycentricCoord) {
	 return barycentricCoord.x >= -0.0001f && //barycentricCoord.x <= 1.0f &&
			barycentricCoord.y >= -0.0001f && //barycentricCoord.y <= 1.0f &&
			barycentricCoord.z >= -0.0001f; // &&barycentricCoord.z <= 1.0f;
 }

 /**
 * For a given barycentric coordinate, compute the corresponding z position
 * (i.e. depth) on the triangle.
 */
__device__ static
float getZAtCoordinate(const float3 &barycentricCoord, const float (&tri)[3]) {
	return (barycentricCoord.x * tri[0]
		+ barycentricCoord.y * tri[1]
		+ barycentricCoord.z * tri[2]);
}

/**
 * Loop over rectangular region covering the triangle and test each pixel for
 * being inside or outside (using bary centric coordinate method). If inside
 * then atomically write to the depth map.
 */
__device__ void drawTriangle(const float (&d)[3], const short2 (&v)[3], const Parameters &params, int* depth_out, int out_pitch4) {
	const int minX = min(v[0].x, min(v[1].x, v[2].x));
	const int minY = min(v[0].y, min(v[1].y, v[2].y));
	const int maxX = max(v[0].x, max(v[1].x, v[2].x));
	const int maxY = max(v[0].y, max(v[1].y, v[2].y));

	// Remove really large triangles
	if ((maxX - minX) * (maxY - minY) <= params.triangle_limit) {
		// TODO: Verify that < is correct, was <= before but < is faster.
		for (int sy=minY; sy < maxY; ++sy) {
			for (int sx=minX; sx < maxX; ++sx) {
				//if () continue;

				float3 baryCentricCoordinate = calculateBarycentricCoordinate(v, make_short2(sx, sy));

				if (sx < params.camera.width && sx >= 0 && sy < params.camera.height && sy >= 0 && isBarycentricCoordInBounds(baryCentricCoordinate)) {
					float new_depth = getZAtCoordinate(baryCentricCoordinate, d);
					atomicMin(&depth_out[sx+sy*out_pitch4], int(new_depth*100000.0f));
				}
			}
		}
	}
}

/**
 * This selects which triangles are drawn. It assumes that discontinuities
 * have already been removed such that the screen coordinate alone acts to
 * indicate a valid or invalid point.
 */
__device__ inline bool isValidTriangle(const short2 (&v)[3]) {
	return v[1].x < 30000 && v[2].x < 30000;
}

/**
 * Read the other two verticies into memory. The template parameters determine
 * which verticies to load.
 */
template <int A, int B>
__device__ bool loadTriangle(int x, int y, float (&d)[3], short2 (&v)[3], const short* __restrict__ depth_in, const short2* __restrict__ screen, int pitch4, int pitch2) {
    d[1] = fixed2float<10>(depth_in[y*pitch2+x+A]);
    d[2] = fixed2float<10>(depth_in[(y+B)*pitch2+x]);
    v[1] = screen[y*pitch4+x+A];
	v[2] = screen[(y+B)*pitch4+x];
	return isValidTriangle(v);
}

/*
 * Convert source screen position to output screen coordinates.
 */
 __global__ void triangle_render_kernel(
        const short* __restrict__ depth_in,
        int* depth_out,
		const short2* __restrict__ screen,
		int width, int height, int pitch2, int pitch4, int out_pitch4, Parameters params) {

	const int x = blockIdx.x*blockDim.x + threadIdx.x;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

    if (x >= 1 && x < width-1 && y >= 1 && y < height-1) {
		float d[3];
		d[0] = fixed2float<10>(depth_in[y*pitch2+x]);

		short2 v[3];
		v[0] = screen[y*pitch4+x];

		if (v[0].x < 30000) {
			// Calculate discontinuity threshold.
			//const float threshold = (params.depthCoef / ((params.depthCoef / d[0]) - params.disconDisparities)) - d[0];

			// Draw (optionally) 4 triangles as a diamond pattern around the central point.
			if (loadTriangle<1,1>(x, y, d, v, depth_in, screen, pitch4, pitch2)) drawTriangle(d, v, params, depth_out, out_pitch4);
			if (loadTriangle<1,-1>(x, y, d, v, depth_in, screen, pitch4, pitch2)) drawTriangle(d, v, params, depth_out, out_pitch4);
			if (loadTriangle<-1,1>(x, y, d, v, depth_in, screen, pitch4, pitch2)) drawTriangle(d, v, params, depth_out, out_pitch4);
			if (loadTriangle<-1,-1>(x, y, d, v, depth_in, screen, pitch4, pitch2)) drawTriangle(d, v, params, depth_out, out_pitch4);
		}
	}
}

void ftl::cuda::triangle_render1(const cv::cuda::GpuMat &depth_in, cv::cuda::GpuMat &depth_out, const cv::cuda::GpuMat &screen, const Parameters &params, cudaStream_t stream) {
	static constexpr int THREADS_X = 8;
	static constexpr int THREADS_Y = 8;

	const dim3 gridSize((depth_in.cols + THREADS_X - 1)/THREADS_X, (depth_in.rows + THREADS_Y - 1)/THREADS_Y);
	const dim3 blockSize(THREADS_X, THREADS_Y);
	
	depth_out.create(params.camera.height, params.camera.width, CV_32S);

	triangle_render_kernel<<<gridSize, blockSize, 0, stream>>>(depth_in.ptr<short>(), depth_out.ptr<int>(), screen.ptr<short2>(), depth_in.cols, depth_in.rows, depth_in.step1(), screen.step1()/2, depth_out.step1(), params);
    cudaSafeCall( cudaGetLastError() );
}

// ==== Merge convert ===========

__global__ void merge_convert_kernel(
		TextureObject<int> depth_in,
		TextureObject<float> depth_out,
		float alpha) {
	const int x = blockIdx.x*blockDim.x + threadIdx.x;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < 0 || x >= depth_in.width() || y < 0 || y >= depth_in.height()) return;

	int d = depth_in.tex2D(x,y);
	float a = float(d)*alpha;
	float b = depth_out.tex2D(x,y);
	depth_out(x,y) = min(a,b);
}

void ftl::cuda::merge_convert_depth(TextureObject<int> &depth_in, TextureObject<float> &depth_out, float alpha, cudaStream_t stream) {
	const dim3 gridSize((depth_in.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth_in.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	merge_convert_kernel<<<gridSize, blockSize, 0, stream>>>(depth_in, depth_out, alpha);
	cudaSafeCall( cudaGetLastError() );
}

// ==== BLENDER ========

/*
 * Merge two depth maps together
 */
 __global__ void mesh_blender_simple_kernel(
        TextureObject<int> depth_in,
		TextureObject<int> depth_out,
		ftl::rgbd::Camera camera,
		float alpha) {
	const int x = blockIdx.x*blockDim.x + threadIdx.x;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

    if (x < 0 || x >= depth_in.width() || y < 0 || y >= depth_in.height()) return;

	int a = depth_in.tex2D(x,y);
	int b = depth_out.tex2D(x,y);

	float mindepth = (float)min(a,b) / 100000.0f;
	float maxdepth = (float)max(a,b) / 100000.0f;
	float weight = ftl::cuda::weighting(maxdepth-mindepth, alpha);

	//depth_out(x,y) = (int)(((float)mindepth + (float)maxdepth*weight) / (1.0f + weight) * 100000.0f);

	float depth = (mindepth + maxdepth*weight) / (1.0f + weight);
	depth_out(x,y) = (int)(depth * 100000.0f);
}

__global__ void mesh_blender_kernel(
		TextureObject<int> depth_in,
		TextureObject<float> depth_out,
		TextureObject<short> weights_in,
		TextureObject<float> weights_out,
		ftl::render::Parameters params,
		ftl::rgbd::Camera camera,
		float4x4 transform,
		float alpha) {

	const int x = blockIdx.x*blockDim.x + threadIdx.x;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	const float d1 = float(depth_in.tex2D((int)x, (int)y)) / 100000.0f;
	const float d2 = depth_out.tex2D((int)x, (int)y);
	const float wout = weights_out.tex2D((int)x, (int)y);

	if (d1 > params.camera.minDepth && d1 < params.camera.maxDepth) {
		//const uint2 rpt = convertScreen<VPMODE>(params, x, y);
		const float3 camPos = transform * params.camera.screenToCam(x, y, d1);
		if (camPos.z > camera.minDepth && camPos.z < camera.maxDepth) {
			const float2 screenPos = camera.camToScreen<float2>(camPos);

			// Not on screen so stop now...
			if (screenPos.x < weights_in.width() && screenPos.y < weights_in.height()) {
				const float win = float(weights_in.tex2D(int(screenPos.x+0.5f), int(screenPos.y+0.5f)));

				if (d1 < d2/wout - alpha || (fabsf(d2/wout - d1) < alpha && win > wout)) {
					depth_out(x,y) = d1 * win;
					weights_out(x,y) = win;
				} //else if (fabsf(d2/wout - d1) < alpha) {
					//depth_out(x,y) = d2 + d1 * win;
					//weights_out(x,y) = wout + win;
					//depth_out(x,y) = (win > wout) ? d1*win : d2;
					//weights_out(x,y) = (win > wout) ? win : wout;
				//}
			}
		}
	}
}

void ftl::cuda::mesh_blender(TextureObject<int> &depth_in, TextureObject<int> &depth_out, const ftl::rgbd::Camera &camera, float alpha, cudaStream_t stream) {
    const dim3 gridSize((depth_in.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth_in.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	mesh_blender_simple_kernel<<<gridSize, blockSize, 0, stream>>>(depth_in, depth_out, camera, alpha);
    cudaSafeCall( cudaGetLastError() );
}

void ftl::cuda::mesh_blender(TextureObject<int> &depth_in, TextureObject<float> &depth_out, TextureObject<short> &weights_in, TextureObject<float> &weights_out, const ftl::render::Parameters &params, const ftl::rgbd::Camera &camera, const float4x4 &transform, float alpha, cudaStream_t stream) {
    const dim3 gridSize((depth_in.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth_in.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	mesh_blender_kernel<<<gridSize, blockSize, 0, stream>>>(depth_in, depth_out, weights_in, weights_out, params, camera, transform, alpha);
    cudaSafeCall( cudaGetLastError() );
}
