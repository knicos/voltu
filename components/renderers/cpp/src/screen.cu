#include <ftl/render/splat_params.hpp>
#include "splatter_cuda.hpp"
#include <ftl/rgbd/camera.hpp>
#include <ftl/cuda_common.hpp>

using ftl::rgbd::Camera;
using ftl::cuda::TextureObject;
using ftl::render::SplatParams;

#define T_PER_BLOCK 8

/*
 * Convert source screen position to output screen coordinates.
 */
 __global__ void screen_coord_kernel(TextureObject<float> depth,
        TextureObject<float> depth_out,
		TextureObject<short2> screen_out, Camera vcamera, float4x4 pose, Camera camera) {
	const int x = blockIdx.x*blockDim.x + threadIdx.x;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x >= 0 && y >= 0 && x < depth.width() && y < depth.height()) {
		uint2 screenPos = make_uint2(30000,30000);
		//screen_out(x,y) = make_short2(screenPos.x, screenPos.y);

		const float d = depth.tex2D(x, y);
		//const float3 worldPos = pose * camera.screenToCam(x,y,d);
		//if (d < camera.minDepth || d > camera.maxDepth) return;

		// Find the virtual screen position of current point
		const float3 camPos =  (d >= camera.minDepth && d <= camera.maxDepth) ? pose * camera.screenToCam(x,y,d) : make_float3(0.0f,0.0f,0.0f); // worldPos;  // params.m_viewMatrix *
		screenPos = vcamera.camToScreen<uint2>(camPos);

		if (	camPos.z < vcamera.minDepth ||
				camPos.z > vcamera.maxDepth ||
				screenPos.x >= vcamera.width ||
				screenPos.y >= vcamera.height)
			screenPos = make_uint2(30000,30000);
		screen_out(x,y) = make_short2(screenPos.x, screenPos.y);
		depth_out(x,y) = camPos.z;
	}
}

void ftl::cuda::screen_coord(TextureObject<float> &depth, TextureObject<float> &depth_out, TextureObject<short2> &screen_out, const SplatParams &params, const float4x4 &pose, const Camera &camera, cudaStream_t stream) {
    const dim3 gridSize((depth.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	screen_coord_kernel<<<gridSize, blockSize, 0, stream>>>(depth, depth_out, screen_out, params.camera, pose, camera);
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
		uint2 screenPos = make_uint2(30000,30000);
		const float d = camera.maxDepth;

		// Find the virtual screen position of current point
		const float3 camPos = pose * camera.screenToCam(x,y,d);
		screenPos = vcamera.camToScreen<uint2>(camPos);

		if (	camPos.z < vcamera.minDepth ||
				camPos.z > vcamera.maxDepth ||
				screenPos.x >= vcamera.width ||
				screenPos.y >= vcamera.height)
			screenPos = make_uint2(30000,30000);

		screen_out(x,y) = make_short2(screenPos.x, screenPos.y);
		depth_out(x,y) = camPos.z;
	}
}

void ftl::cuda::screen_coord(TextureObject<float> &depth_out, TextureObject<short2> &screen_out, const SplatParams &params, const float4x4 &pose, const Camera &camera, cudaStream_t stream) {
	const dim3 gridSize((screen_out.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (screen_out.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	screen_coord_kernel<<<gridSize, blockSize, 0, stream>>>(depth_out, screen_out, params.camera, pose, camera);
	cudaSafeCall( cudaGetLastError() );
}
