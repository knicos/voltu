#include <ftl/render/render_params.hpp>
#include "splatter_cuda.hpp"
#include <ftl/rgbd/camera.hpp>
#include <ftl/cuda_common.hpp>

using ftl::rgbd::Camera;
using ftl::cuda::TextureObject;
using ftl::render::Parameters;
using ftl::render::ViewPortMode;

#define T_PER_BLOCK 8

__device__ inline uint2 make_uint2(const float2 &f) {
	return {static_cast<uint>(f.x), static_cast<uint>(f.y)};
}

template <ViewPortMode VPMODE>
__device__ inline uint2 convertToScreen(const Parameters &params, const float3 &camPos);

template <>
__device__ inline uint2 convertToScreen<ViewPortMode::Disabled>(const Parameters &params, const float3 &camPos) {
	return  params.camera.camToScreen<uint2>(camPos);
}

template <>
__device__ inline uint2 convertToScreen<ViewPortMode::Clipping>(const Parameters &params, const float3 &camPos) {
	const uint2 r = params.camera.camToScreen<uint2>(camPos);
	return (params.viewport.inside(r.x, r.y)) ? r : make_uint2(30000, 30000);
}

template <>
__device__ inline uint2 convertToScreen<ViewPortMode::Stretch>(const Parameters &params, const float3 &camPos) {
	return make_uint2(params.viewport.map(params.camera, params.camera.camToScreen<float2>(camPos)));
}

/*template <>
__device__ inline uint2 convertToScreen<ViewPortMode::Warping>(const Parameters &params, const float3 &camPos) {
	float2 pt =  params.camera.camToScreen<float2>(camPos); //params.viewport.map(params.camera, params.camera.camToScreen<float2>(camPos));
	const float coeff = 1.0f / (params.viewport.warpMatrix.entries[6] * pt.x + params.viewport.warpMatrix.entries[7] * pt.y + params.viewport.warpMatrix.entries[8]);
	const float xcoo = coeff * (params.viewport.warpMatrix.entries[0] * pt.x + params.viewport.warpMatrix.entries[1] * pt.y + params.viewport.warpMatrix.entries[2]);
	const float ycoo = coeff * (params.viewport.warpMatrix.entries[3] * pt.x + params.viewport.warpMatrix.entries[4] * pt.y + params.viewport.warpMatrix.entries[5]);
	return make_uint2(xcoo, ycoo);
}*/

/*
 * Convert source screen position to output screen coordinates.
 */
 template <ftl::render::ViewPortMode VPMODE>
 __global__ void screen_coord_kernel(TextureObject<float> depth,
        TextureObject<float> depth_out,
		TextureObject<short2> screen_out, Parameters params, float4x4 pose, Camera camera) {
	const int x = blockIdx.x*blockDim.x + threadIdx.x;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x >= 0 && y >= 0 && x < depth.width() && y < depth.height()) {
		uint2 screenPos = make_uint2(30000,30000);

		const float d = depth.tex2D(x, y);

		// Find the virtual screen position of current point
		const float3 camPos =  (d > camera.minDepth && d < camera.maxDepth) ? pose * camera.screenToCam(x,y,d) : make_float3(0.0f,0.0f,0.0f);
		screenPos = convertToScreen<VPMODE>(params, camPos);

		if (	camPos.z < params.camera.minDepth ||
				camPos.z > params.camera.maxDepth ||
				//!vp.inside(screenPos.x, screenPos.y))
				screenPos.x >= params.camera.width ||
				screenPos.y >= params.camera.height)
			screenPos = make_uint2(30000,30000);
		screen_out(x,y) = make_short2(screenPos.x, screenPos.y);
		depth_out(x,y) = camPos.z;
	}
}

void ftl::cuda::screen_coord(TextureObject<float> &depth, TextureObject<float> &depth_out,
		TextureObject<short2> &screen_out, const Parameters &params,
		const float4x4 &pose, const Camera &camera, cudaStream_t stream) {
    const dim3 gridSize((depth.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	switch (params.viewPortMode) {
	case ViewPortMode::Disabled: screen_coord_kernel<ViewPortMode::Disabled><<<gridSize, blockSize, 0, stream>>>(depth, depth_out, screen_out, params, pose, camera); break;
	case ViewPortMode::Clipping: screen_coord_kernel<ViewPortMode::Clipping><<<gridSize, blockSize, 0, stream>>>(depth, depth_out, screen_out, params, pose, camera); break;
	case ViewPortMode::Stretch: screen_coord_kernel<ViewPortMode::Stretch><<<gridSize, blockSize, 0, stream>>>(depth, depth_out, screen_out, params, pose, camera); break;
	}
	cudaSafeCall( cudaGetLastError() );
}


// ==== Constant depth version =================================================

/*
 * Convert source screen position to output screen coordinates. Assumes a
 * constant depth of 1m instead of using a depth channel input.
 */
 __global__ void screen_coord_kernel(TextureObject<float> depth_out,
		TextureObject<short2> screen_out, Camera vcamera, float4x4 pose, Camera camera) {
	const int x = blockIdx.x*blockDim.x + threadIdx.x;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x >= 0 && y >= 0 && x < depth_out.width() && y < depth_out.height()) {
		//uint2 screenPos = make_uint2(30000,30000);
		const float d = camera.maxDepth;

		// Find the virtual screen position of current point
		const float3 camPos = pose * camera.screenToCam(x,y,d);
		uint2 screenPos = vcamera.camToScreen<uint2>(camPos);

		if (	camPos.z < vcamera.minDepth ||
				camPos.z > vcamera.maxDepth ||
				screenPos.x >= vcamera.width ||
				screenPos.y >= vcamera.height)
			screenPos = make_uint2(30000,30000);

		screen_out(x,y) = make_short2(screenPos.x, screenPos.y);
		depth_out(x,y) = camPos.z;
	}
}

void ftl::cuda::screen_coord(TextureObject<float> &depth_out, TextureObject<short2> &screen_out, const Parameters &params, const float4x4 &pose, const Camera &camera, cudaStream_t stream) {
	const dim3 gridSize((screen_out.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (screen_out.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	screen_coord_kernel<<<gridSize, blockSize, 0, stream>>>(depth_out, screen_out, params.camera, pose, camera);
	cudaSafeCall( cudaGetLastError() );
}
