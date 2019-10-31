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
		TextureObject<int> depth_in,        // Virtual depth map
		TextureObject<float4> normals,
		TextureObject<B> out,			// Accumulated output
		TextureObject<float> contrib,
		SplatParams params,
		Camera camera, float4x4 poseInv) {
        
	const int x = (blockIdx.x*blockDim.x + threadIdx.x);
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	const float d = (float)depth_in.tex2D((int)x, (int)y) / 100000.0f;
	if (d < params.camera.minDepth || d > params.camera.maxDepth) return;

	const float3 worldPos = params.m_viewMatrixInverse * params.camera.screenToCam(x, y, d);
	//if (worldPos.x == MINF || (!(params.m_flags & ftl::render::kShowDisconMask) && worldPos.w < 0.0f)) return;

	const float3 camPos = poseInv * worldPos;
	if (camPos.z < camera.minDepth) return;
	if (camPos.z > camera.maxDepth) return;
	const uint2 screenPos = camera.camToScreen<uint2>(camPos);

	// Not on screen so stop now...
	if (screenPos.x >= depth_src.width() || screenPos.y >= depth_src.height()) return;
            
	// Calculate the dot product of surface normal and camera ray
	const float3 n = poseInv.getFloat3x3() * make_float3(normals.tex2D((int)x, (int)y));
	float3 ray = camera.screenToCam(screenPos.x, screenPos.y, 1.0f);
	ray = ray / length(ray);
	const float dotproduct = max(dot(ray,n),0.0f);
    
	const float d2 = depth_src.tex2D((int)screenPos.x, (int)screenPos.y);
	const A input = in.tex2D((int)screenPos.x, (int)screenPos.y); //generateInput(in.tex2D((int)screenPos.x, (int)screenPos.y), params, worldPos);
	float weight = ftl::cuda::weighting(fabs(camPos.z - d2), 0.02f);

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
		TextureObject<int> &depth_in,        // Virtual depth map
		TextureObject<float4> &normals,
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
		normals,
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
	ftl::cuda::TextureObject<int> &depth_in,		// Virtual depth map
	ftl::cuda::TextureObject<float4> &normals,
	ftl::cuda::TextureObject<float4> &out,	// Accumulated output
	ftl::cuda::TextureObject<float> &contrib,
	const ftl::render::SplatParams &params,
	const ftl::rgbd::Camera &camera,
	const float4x4 &poseInv, cudaStream_t stream);

template void ftl::cuda::reproject(
		ftl::cuda::TextureObject<float> &in,	// Original colour image
		ftl::cuda::TextureObject<float> &depth_src,		// Original 3D points
		ftl::cuda::TextureObject<int> &depth_in,		// Virtual depth map
		ftl::cuda::TextureObject<float4> &normals,
		ftl::cuda::TextureObject<float> &out,	// Accumulated output
		ftl::cuda::TextureObject<float> &contrib,
		const ftl::render::SplatParams &params,
		const ftl::rgbd::Camera &camera,
		const float4x4 &poseInv, cudaStream_t stream);

template void ftl::cuda::reproject(
		ftl::cuda::TextureObject<float4> &in,	// Original colour image
		ftl::cuda::TextureObject<float> &depth_src,		// Original 3D points
		ftl::cuda::TextureObject<int> &depth_in,		// Virtual depth map
		ftl::cuda::TextureObject<float4> &normals,
		ftl::cuda::TextureObject<float4> &out,	// Accumulated output
		ftl::cuda::TextureObject<float> &contrib,
		const ftl::render::SplatParams &params,
		const ftl::rgbd::Camera &camera,
		const float4x4 &poseInv, cudaStream_t stream);

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
		TextureObject<int> depth_in,        // Virtual depth map
		TextureObject<B> out,			// Accumulated output
		TextureObject<float> contrib,
		SplatParams params,
		Camera camera, float4x4 poseInv) {
        
	const int x = (blockIdx.x*blockDim.x + threadIdx.x);
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	const float d = (float)depth_in.tex2D((int)x, (int)y) / 100000.0f;
	if (d < params.camera.minDepth || d > params.camera.maxDepth) return;

	const float3 worldPos = params.m_viewMatrixInverse * params.camera.screenToCam(x, y, d);
	//if (worldPos.x == MINF || (!(params.m_flags & ftl::render::kShowDisconMask) && worldPos.w < 0.0f)) return;

	const float3 camPos = poseInv * worldPos;
	if (camPos.z < camera.minDepth) return;
	if (camPos.z > camera.maxDepth) return;
	const uint2 screenPos = camera.camToScreen<uint2>(camPos);

	// Not on screen so stop now...
	if (screenPos.x >= depth_src.width() || screenPos.y >= depth_src.height()) return;
    
	const float d2 = depth_src.tex2D((int)screenPos.x, (int)screenPos.y);
	const A input = in.tex2D((int)screenPos.x, (int)screenPos.y); //generateInput(in.tex2D((int)screenPos.x, (int)screenPos.y), params, worldPos);
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
		TextureObject<int> &depth_in,        // Virtual depth map
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
	ftl::cuda::TextureObject<int> &depth_in,		// Virtual depth map
	ftl::cuda::TextureObject<float4> &out,	// Accumulated output
	ftl::cuda::TextureObject<float> &contrib,
	const ftl::render::SplatParams &params,
	const ftl::rgbd::Camera &camera,
	const float4x4 &poseInv, cudaStream_t stream);

template void ftl::cuda::reproject(
		ftl::cuda::TextureObject<float> &in,	// Original colour image
		ftl::cuda::TextureObject<float> &depth_src,		// Original 3D points
		ftl::cuda::TextureObject<int> &depth_in,		// Virtual depth map
		ftl::cuda::TextureObject<float> &out,	// Accumulated output
		ftl::cuda::TextureObject<float> &contrib,
		const ftl::render::SplatParams &params,
		const ftl::rgbd::Camera &camera,
		const float4x4 &poseInv, cudaStream_t stream);

template void ftl::cuda::reproject(
		ftl::cuda::TextureObject<float4> &in,	// Original colour image
		ftl::cuda::TextureObject<float> &depth_src,		// Original 3D points
		ftl::cuda::TextureObject<int> &depth_in,		// Virtual depth map
		ftl::cuda::TextureObject<float4> &out,	// Accumulated output
		ftl::cuda::TextureObject<float> &contrib,
		const ftl::render::SplatParams &params,
		const ftl::rgbd::Camera &camera,
		const float4x4 &poseInv, cudaStream_t stream);
