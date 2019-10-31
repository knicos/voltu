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
		TextureObject<short2> screen_out, SplatParams params, float4x4 pose, Camera camera) {
	const int x = blockIdx.x*blockDim.x + threadIdx.x;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

    uint2 screenPos = make_uint2(30000,30000);
    screen_out(x,y) = make_short2(screenPos.x, screenPos.y);

    const float d = depth.tex2D(x, y);
	const float3 worldPos = pose * camera.screenToCam(x,y,d);
	if (d < camera.minDepth || d > camera.maxDepth) return;

    // Find the virtual screen position of current point
	const float3 camPos = params.m_viewMatrix * worldPos;
    screenPos = params.camera.camToScreen<uint2>(camPos);

    if (camPos.z < params.camera.minDepth || camPos.z > params.camera.maxDepth || screenPos.x >= params.camera.width || screenPos.y >= params.camera.height)
        screenPos = make_uint2(30000,30000);
    screen_out(x,y) = make_short2(screenPos.x, screenPos.y);
    depth_out(x,y) = camPos.z;
}

void ftl::cuda::screen_coord(TextureObject<float> &depth, TextureObject<float> &depth_out, TextureObject<short2> &screen_out, const SplatParams &params, const float4x4 &pose, const Camera &camera, cudaStream_t stream) {
    const dim3 gridSize((depth.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	screen_coord_kernel<<<gridSize, blockSize, 0, stream>>>(depth, depth_out, screen_out, params, pose, camera);
    cudaSafeCall( cudaGetLastError() );
}
