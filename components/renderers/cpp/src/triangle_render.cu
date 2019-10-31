#include <ftl/render/splat_params.hpp>
#include "splatter_cuda.hpp"
#include <ftl/rgbd/camera.hpp>
#include <ftl/cuda_common.hpp>

using ftl::rgbd::Camera;
using ftl::cuda::TextureObject;
using ftl::render::SplatParams;

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
	 float alpha = 1.0 - beta - gamma;
	 return make_float3(alpha, beta, gamma);
 }
 
 /**
  * Check if a barycentric coordinate is within the boundaries of a triangle.
  */
 __host__ __device__ static
 bool isBarycentricCoordInBounds(const float3 &barycentricCoord) {
	 return barycentricCoord.x >= 0.0 && barycentricCoord.x <= 1.0 &&
			barycentricCoord.y >= 0.0 && barycentricCoord.y <= 1.0 &&
			barycentricCoord.z >= 0.0 && barycentricCoord.z <= 1.0;
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

/*
 * Convert source screen position to output screen coordinates.
 */
 template <int A, int B>
 __global__ void triangle_render_1_kernel(
        TextureObject<float> depth_in,
        TextureObject<int> depth_out,
		TextureObject<short2> screen, SplatParams params) {
	const int x = blockIdx.x*blockDim.x + threadIdx.x;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

    if (x < 1 || x >= depth_in.width()-1 || y < 1 || y >= depth_in.height()-1) return;

    float d[3];
    d[0] = depth_in.tex2D(x,y);
    d[1] = depth_in.tex2D(x+A,y);
    d[2] = depth_in.tex2D(x,y+B);

    // Is this triangle valid
	if (fabs(d[0] - d[1]) > 0.04f || fabs(d[0] - d[2]) > 0.04f) return;
	if (d[0] < params.camera.minDepth || d[0] > params.camera.maxDepth) return;

    short2 v[3];
    v[0] = screen.tex2D(x,y);
    v[1] = screen.tex2D(x+A,y);
	v[2] = screen.tex2D(x,y+B);

	// Attempt to back face cull, but not great
	//if ((v[1].x - v[0].x) * A < 0 || (v[2].y - v[0].y) * B < 0) return;

	const int minX = min(v[0].x, min(v[1].x, v[2].x));
	const int minY = min(v[0].y, min(v[1].y, v[2].y));
	const int maxX = max(v[0].x, max(v[1].x, v[2].x));
	const int maxY = max(v[0].y, max(v[1].y, v[2].y));

	// Remove really large triangles
	if ((maxX - minX) * (maxY - minY) > 200) return;

	for (int sy=minY; sy <= maxY; ++sy) {
		for (int sx=minX; sx <= maxX; ++sx) {
			if (sx >= params.camera.width || sx < 0 || sy >= params.camera.height || sy < 0) continue;

			float3 baryCentricCoordinate = calculateBarycentricCoordinate(v, make_short2(sx, sy));

			if (isBarycentricCoordInBounds(baryCentricCoordinate)) {
				float new_depth = getZAtCoordinate(baryCentricCoordinate, d);
				atomicMin(&depth_out(sx,sy), int(new_depth*100000.0f));
			}
		}
	}
}

void ftl::cuda::triangle_render1(TextureObject<float> &depth_in, TextureObject<int> &depth_out, TextureObject<short2> &screen, const SplatParams &params, cudaStream_t stream) {
    const dim3 gridSize((depth_in.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth_in.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	triangle_render_1_kernel<1,1><<<gridSize, blockSize, 0, stream>>>(depth_in, depth_out, screen, params);
	triangle_render_1_kernel<1,-1><<<gridSize, blockSize, 0, stream>>>(depth_in, depth_out, screen, params);
	triangle_render_1_kernel<-1,1><<<gridSize, blockSize, 0, stream>>>(depth_in, depth_out, screen, params);
	triangle_render_1_kernel<-1,-1><<<gridSize, blockSize, 0, stream>>>(depth_in, depth_out, screen, params);
    cudaSafeCall( cudaGetLastError() );
}
