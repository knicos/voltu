#include <ftl/render/splat_params.hpp>
#include "splatter_cuda.hpp"
#include <ftl/rgbd/camera.hpp>
#include <ftl/cuda_common.hpp>

#include <ftl/cuda/weighting.hpp>
#include <ftl/cuda/makers.hpp>

#define T_PER_BLOCK 8
#define ACCUM_DIAMETER 8

using ftl::cuda::TextureObject;
using ftl::render::SplatParams;
using ftl::rgbd::Camera;

/*template <typename T>
__device__ inline T generateInput(const T &in, const SplatParams &params, const float4 &worldPos) {
	return in;
}

template <>
__device__ inline uchar4 generateInput(const uchar4 &in, const SplatParams &params, const float4 &worldPos) {
	return (params.m_flags & ftl::render::kShowDisconMask && worldPos.w < 0.0f) ?
		make_uchar4(0,0,255,255) :  // Show discontinuity mask in red
		in;
}*/

template <typename A, typename B>
__device__ inline B weightInput(const A &in, float weight) {
	return in * weight;
}

template <>
__device__ inline float4 weightInput(const uchar4 &in, float weight) {
	return make_float4(
		(float)in.x * weight,
		(float)in.y * weight,
		(float)in.z * weight,
		(float)in.w * weight);
}

template <typename T>
__device__ inline void accumulateOutput(TextureObject<T> &out, TextureObject<float> &contrib, const uint2 &pos, const T &in, float w) {
	atomicAdd(&out(pos.x, pos.y), in);
	atomicAdd(&contrib(pos.x, pos.y), w);
} 

template <>
__device__ inline void accumulateOutput(TextureObject<float4> &out, TextureObject<float> &contrib, const uint2 &pos, const float4 &in, float w) {
	atomicAdd((float*)&out(pos.x, pos.y), in.x);
	atomicAdd(((float*)&out(pos.x, pos.y))+1, in.y);
	atomicAdd(((float*)&out(pos.x, pos.y))+2, in.z);
	atomicAdd(((float*)&out(pos.x, pos.y))+3, in.w);
	atomicAdd(&contrib(pos.x, pos.y), w);
} 

/*
 * Pass 2: Accumulate attribute contributions if the points pass a visibility test.
 */
 template <typename A, typename B>
__global__ void reprojection_kernel(
        TextureObject<A> in,				// Attribute input
        TextureObject<float> depth_src,
		TextureObject<float> depth_in,        // Virtual depth map
		TextureObject<float4> normals,
		TextureObject<B> out,			// Accumulated output
		TextureObject<float> contrib,
		SplatParams params,
		Camera camera, float4x4 transform, float3x3 transformR) {
        
	const int x = (blockIdx.x*blockDim.x + threadIdx.x);
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	const float d = depth_in.tex2D((int)x, (int)y);
	if (d < params.camera.minDepth || d > params.camera.maxDepth) return;

	//const float3 worldPos = params.m_viewMatrixInverse * params.camera.screenToCam(x, y, d);
	//if (worldPos.x == MINF || (!(params.m_flags & ftl::render::kShowDisconMask) && worldPos.w < 0.0f)) return;

	const float3 camPos = transform * params.camera.screenToCam(x, y, d);
	if (camPos.z < camera.minDepth) return;
	if (camPos.z > camera.maxDepth) return;
	const float2 screenPos = camera.camToScreen<float2>(camPos);

	// Not on screen so stop now...
	if (screenPos.x >= depth_src.width() || screenPos.y >= depth_src.height()) return;
            
	// Calculate the dot product of surface normal and camera ray
	const float3 n = transformR * make_float3(normals.tex2D((int)x, (int)y));
	float3 ray = camera.screenToCam(screenPos.x, screenPos.y, 1.0f);
	ray = ray / length(ray);

	// Allow slightly beyond 90 degrees due to normal estimation errors
	const float dotproduct = (max(dot(ray,n),-0.1f)+0.1) / 1.1f;
    
	const float d2 = depth_src.tex2D(int(screenPos.x+0.5f), int(screenPos.y+0.5f));

	const float inSX = float(in.width()) / float(depth_src.width());
	const float inSY = float(in.height()) / float(depth_src.height());
	const auto input = in.tex2D(screenPos.x*inSX, screenPos.y*inSY); //generateInput(in.tex2D((int)screenPos.x, (int)screenPos.y), params, worldPos);

	// TODO: Z checks need to interpolate between neighbors if large triangles are used
	//float weight = ftl::cuda::weighting(fabs(camPos.z - d2), params.depthThreshold);
	float weight = (fabs(camPos.z - d2) <= params.depthThreshold) ? 1.0f : 0.0f;

	/* Buehler C. et al. 2001. Unstructured Lumigraph Rendering. */
	/* Orts-Escolano S. et al. 2016. Holoportation: Virtual 3D teleportation in real-time. */
	// This is the simple naive colour weighting. It might be good
	// enough for our purposes if the alignment step prevents ghosting
	// TODO: Use depth and perhaps the neighbourhood consistency in:
	//     Kuster C. et al. 2011. FreeCam: A hybrid camera system for interactive free-viewpoint video
	if (params.m_flags & ftl::render::kNormalWeightColours) weight *= dotproduct;

	const B weighted = make<B>(input) * weight; //weightInput(input, weight);

	if (weight > 0.0f) {
		accumulateOutput(out, contrib, make_uint2(x,y), weighted, weight);
		//out(screenPos.x, screenPos.y) = input;
	}
}


template <typename A, typename B>
void ftl::cuda::reproject(
        TextureObject<A> &in,
        TextureObject<float> &depth_src,       // Original 3D points
		TextureObject<float> &depth_in,        // Virtual depth map
		TextureObject<float4> &normals,
		TextureObject<B> &out,   // Accumulated output
		TextureObject<float> &contrib,
		const SplatParams &params,
		const Camera &camera, const float4x4 &transform, const float3x3 &transformR, cudaStream_t stream) {
	const dim3 gridSize((out.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (out.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    reprojection_kernel<<<gridSize, blockSize, 0, stream>>>(
        in,
        depth_src,
		depth_in,
		normals,
		out,
		contrib,
		params,
		camera,
		transform,
		transformR
    );
    cudaSafeCall( cudaGetLastError() );
}

template void ftl::cuda::reproject(
	ftl::cuda::TextureObject<uchar4> &in,	// Original colour image
	ftl::cuda::TextureObject<float> &depth_src,		// Original 3D points
	ftl::cuda::TextureObject<float> &depth_in,		// Virtual depth map
	ftl::cuda::TextureObject<float4> &normals,
	ftl::cuda::TextureObject<float4> &out,	// Accumulated output
	ftl::cuda::TextureObject<float> &contrib,
	const ftl::render::SplatParams &params,
	const ftl::rgbd::Camera &camera,
	const float4x4 &transform, const float3x3 &transformR, cudaStream_t stream);

template void ftl::cuda::reproject(
		ftl::cuda::TextureObject<float> &in,	// Original colour image
		ftl::cuda::TextureObject<float> &depth_src,		// Original 3D points
		ftl::cuda::TextureObject<float> &depth_in,		// Virtual depth map
		ftl::cuda::TextureObject<float4> &normals,
		ftl::cuda::TextureObject<float> &out,	// Accumulated output
		ftl::cuda::TextureObject<float> &contrib,
		const ftl::render::SplatParams &params,
		const ftl::rgbd::Camera &camera,
		const float4x4 &transform, const float3x3 &transformR, cudaStream_t stream);

template void ftl::cuda::reproject(
		ftl::cuda::TextureObject<float4> &in,	// Original colour image
		ftl::cuda::TextureObject<float> &depth_src,		// Original 3D points
		ftl::cuda::TextureObject<float> &depth_in,		// Virtual depth map
		ftl::cuda::TextureObject<float4> &normals,
		ftl::cuda::TextureObject<float4> &out,	// Accumulated output
		ftl::cuda::TextureObject<float> &contrib,
		const ftl::render::SplatParams &params,
		const ftl::rgbd::Camera &camera,
		const float4x4 &transform, const float3x3 &transformR, cudaStream_t stream);

//==============================================================================
//  Without normals
//==============================================================================

/*
 * Pass 2: Accumulate attribute contributions if the points pass a visibility test.
 */
 template <typename A, typename B>
__global__ void reprojection_kernel(
        TextureObject<A> in,				// Attribute input
        TextureObject<float> depth_src,
		TextureObject<float> depth_in,        // Virtual depth map
		TextureObject<B> out,			// Accumulated output
		TextureObject<float> contrib,
		SplatParams params,
		Camera camera, float4x4 poseInv) {
        
	const int x = (blockIdx.x*blockDim.x + threadIdx.x);
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	const float d = depth_in.tex2D((int)x, (int)y);
	if (d < params.camera.minDepth || d > params.camera.maxDepth) return;

	//const float3 worldPos = params.m_viewMatrixInverse * params.camera.screenToCam(x, y, d);
	//if (worldPos.x == MINF || (!(params.m_flags & ftl::render::kShowDisconMask) && worldPos.w < 0.0f)) return;

	const float3 camPos = poseInv * params.camera.screenToCam(x, y, d);
	if (camPos.z < camera.minDepth) return;
	if (camPos.z > camera.maxDepth) return;
	const float2 screenPos = camera.camToScreen<float2>(camPos);

	// Not on screen so stop now...
	if (screenPos.x >= depth_src.width() || screenPos.y >= depth_src.height()) return;
    
	const float d2 = depth_src.tex2D((int)(screenPos.x+0.5f), (int)(screenPos.y+0.5f));

	const float inSX = float(in.width()) / float(depth_src.width());
	const float inSY = float(in.height()) / float(depth_src.height());
	const auto input = in.tex2D(screenPos.x*inSX, screenPos.y*inSY); //generateInput(in.tex2D((int)screenPos.x, (int)screenPos.y), params, worldPos);

	float weight = ftl::cuda::weighting(fabs(camPos.z - d2), 0.02f);
	const B weighted = make<B>(input) * weight;

	if (weight > 0.0f) {
		accumulateOutput(out, contrib, make_uint2(x,y), weighted, weight);
		//out(screenPos.x, screenPos.y) = input;
	}
}


template <typename A, typename B>
void ftl::cuda::reproject(
        TextureObject<A> &in,
        TextureObject<float> &depth_src,       // Original 3D points
		TextureObject<float> &depth_in,        // Virtual depth map
		TextureObject<B> &out,   // Accumulated output
		TextureObject<float> &contrib,
		const SplatParams &params,
		const Camera &camera, const float4x4 &poseInv, cudaStream_t stream) {
	const dim3 gridSize((out.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (out.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    reprojection_kernel<<<gridSize, blockSize, 0, stream>>>(
        in,
        depth_src,
		depth_in,
		out,
		contrib,
		params,
		camera,
		poseInv
    );
    cudaSafeCall( cudaGetLastError() );
}

template void ftl::cuda::reproject(
	ftl::cuda::TextureObject<uchar4> &in,	// Original colour image
	ftl::cuda::TextureObject<float> &depth_src,		// Original 3D points
	ftl::cuda::TextureObject<float> &depth_in,		// Virtual depth map
	ftl::cuda::TextureObject<float4> &out,	// Accumulated output
	ftl::cuda::TextureObject<float> &contrib,
	const ftl::render::SplatParams &params,
	const ftl::rgbd::Camera &camera,
	const float4x4 &poseInv, cudaStream_t stream);

template void ftl::cuda::reproject(
		ftl::cuda::TextureObject<float> &in,	// Original colour image
		ftl::cuda::TextureObject<float> &depth_src,		// Original 3D points
		ftl::cuda::TextureObject<float> &depth_in,		// Virtual depth map
		ftl::cuda::TextureObject<float> &out,	// Accumulated output
		ftl::cuda::TextureObject<float> &contrib,
		const ftl::render::SplatParams &params,
		const ftl::rgbd::Camera &camera,
		const float4x4 &poseInv, cudaStream_t stream);

template void ftl::cuda::reproject(
		ftl::cuda::TextureObject<float4> &in,	// Original colour image
		ftl::cuda::TextureObject<float> &depth_src,		// Original 3D points
		ftl::cuda::TextureObject<float> &depth_in,		// Virtual depth map
		ftl::cuda::TextureObject<float4> &out,	// Accumulated output
		ftl::cuda::TextureObject<float> &contrib,
		const ftl::render::SplatParams &params,
		const ftl::rgbd::Camera &camera,
		const float4x4 &poseInv, cudaStream_t stream);


// ===== Equirectangular Reprojection ==========================================

__device__ inline float2 equirect_reprojection(int x_img, int y_img, double f, const float3x3 &rot, int w1, int h1, const ftl::rgbd::Camera &cam) {
	float3 ray3d = cam.screenToCam(x_img, y_img, 1.0f);
	ray3d /= length(ray3d);
	ray3d = rot * ray3d;

    //inverse formula for spherical projection, reference Szeliski book "Computer Vision: Algorithms and Applications" p439.
    float theta = atan2(ray3d.y,sqrt(ray3d.x*ray3d.x+ray3d.z*ray3d.z));
	float phi = atan2(ray3d.x, ray3d.z);
	
	const float pi = 3.14f;

    //get 2D point on equirectangular map
    float x_sphere = (((phi*w1)/pi+w1)/2); 
    float y_sphere = (theta+ pi/2)*h1/pi;

    return make_float2(x_sphere,y_sphere);
};

__global__ void equirectangular_kernel(
		TextureObject<uchar4> image_in,
		TextureObject<uchar4> image_out,
		Camera camera, float3x3 pose) {
		
	const int x = (blockIdx.x*blockDim.x + threadIdx.x);
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x >= 0 && y >= 0 && x < image_out.width() && y < image_out.height()) {
		const float2 p = equirect_reprojection(x,y, camera.fx, pose, image_in.width(), image_in.height(), camera);
		const float4 colour = image_in.tex2D(p.x, p.y);
		image_out(x,y) = make_uchar4(colour.x, colour.y, colour.z, 0);
	}
}

void ftl::cuda::equirectangular_reproject(
		ftl::cuda::TextureObject<uchar4> &image_in,
		ftl::cuda::TextureObject<uchar4> &image_out,
		const ftl::rgbd::Camera &camera, const float3x3 &pose, cudaStream_t stream) {

	const dim3 gridSize((image_out.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (image_out.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	equirectangular_kernel<<<gridSize, blockSize, 0, stream>>>(image_in, image_out, camera, pose);
	cudaSafeCall( cudaGetLastError() );
}

// ==== Correct for bad colours ================================================

__device__ inline uchar4 make_uchar4(const float4 v) {
	return make_uchar4(v.x,v.y,v.z,v.w);
}

template <int RADIUS>
__global__ void fix_colour_kernel(
		TextureObject<float> depth,
		TextureObject<uchar4> out,
		TextureObject<float> contribs,
		uchar4 bad_colour,
		ftl::rgbd::Camera cam) {
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < out.width() && y < out.height()) {
		const float contrib = contribs.tex2D((int)x,(int)y);
		const float d = depth.tex2D(x,y);

		if (contrib < 0.0000001f && d > cam.minDepth && d < cam.maxDepth) {
			float4 sumcol = make_float4(0.0f);
			float count = 0.0f;

			for (int v=-RADIUS; v<=RADIUS; ++v) {
				for (int u=-RADIUS; u<=RADIUS; ++u) {
					const float contrib = contribs.tex2D((int)x+u,(int)y+v);
					const float4 c = make_float4(out(int(x)+u,int(y)+v));
					if (contrib > 0.0000001f) {
						sumcol += c;
						count += 1.0f;
					}
				}
			}

			out(x,y) = (count > 0.0f) ? make_uchar4(sumcol / count) : bad_colour;
		}
	}
}

void ftl::cuda::fix_bad_colour(
		TextureObject<float> &depth,
		TextureObject<uchar4> &out,
		TextureObject<float> &contribs,
		uchar4 bad_colour,
		const ftl::rgbd::Camera &cam,
		cudaStream_t stream) {
	const dim3 gridSize((out.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (out.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	fix_colour_kernel<1><<<gridSize, blockSize, 0, stream>>>(depth, out, contribs, bad_colour, cam);
	cudaSafeCall( cudaGetLastError() );
}
