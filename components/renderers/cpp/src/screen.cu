#include <ftl/render/render_params.hpp>
#include "splatter_cuda.hpp"
#include <ftl/rgbd/camera.hpp>
#include <ftl/cuda_common.hpp>
#include <cudatl/fixed.hpp>

using ftl::rgbd::Camera;
using ftl::cuda::TextureObject;
using ftl::render::Parameters;
using ftl::render::ViewPortMode;
using ftl::rgbd::Projection;

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

/*
 * Convert source screen position to output screen coordinates.
 */
 template <ftl::render::ViewPortMode VPMODE, Projection PROJECT>
 __global__ void screen_coord_kernel(
	const float* __restrict__ depth,
	short* __restrict__ depth_out,
	short2* __restrict__ screen_out,
    int pitch4, int pitch2, Parameters params, float4x4 pose, Camera camera)
{
	const int x = blockIdx.x*blockDim.x + threadIdx.x;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x >= 0 && y >= 0 && x < camera.width && y < camera.height) {
		//uint2 screenPos = make_uint2(30000,30000);

		const float d = depth[y*pitch4+x];

		// Find the virtual screen position of current point
		const float3 camPos =  (d > camera.minDepth && d < camera.maxDepth) ? pose * camera.screenToCam(x,y,d) : make_float3(0.0f,0.0f,0.0f);
		float3 screenPos = params.camera.project<PROJECT>(camPos); //convertToScreen<VPMODE>(params, camPos);

		if (	screenPos.z < params.camera.minDepth ||
				screenPos.z > params.camera.maxDepth ||
				//!vp.inside(screenPos.x, screenPos.y))
				screenPos.x < 0.0f || screenPos.y < 0.0f ||
				screenPos.x >= params.camera.width ||
				screenPos.y >= params.camera.height)
			screenPos = make_float3(30000,30000,0);

		screen_out[y*pitch4+x] = make_short2(screenPos.x, screenPos.y);
		depth_out[y*pitch2+x] = cudatl::float2fixed<10>(screenPos.z);
	}
}

void ftl::cuda::screen_coord(const cv::cuda::GpuMat &depth, cv::cuda::GpuMat &depth_out,
		cv::cuda::GpuMat &screen_out, const Parameters &params,
		const float4x4 &pose, const Camera &camera, cudaStream_t stream) {

	static constexpr int THREADS_X = 8;
	static constexpr int THREADS_Y = 8;

    const dim3 gridSize((depth.cols + THREADS_X - 1)/THREADS_X, (depth.rows + THREADS_Y - 1)/THREADS_Y);
	const dim3 blockSize(THREADS_X, THREADS_Y);
	
	depth_out.create(depth.size(), CV_16S);
	screen_out.create(depth.size(), CV_16SC2);

	if (params.projection == Projection::PERSPECTIVE) {
		switch (params.viewPortMode) {
		case ViewPortMode::Disabled: screen_coord_kernel<ViewPortMode::Disabled, Projection::PERSPECTIVE><<<gridSize, blockSize, 0, stream>>>(depth.ptr<float>(), depth_out.ptr<short>(), screen_out.ptr<short2>(), depth.step1(), depth_out.step1(), params, pose, camera); break;
		case ViewPortMode::Clipping: screen_coord_kernel<ViewPortMode::Clipping, Projection::PERSPECTIVE><<<gridSize, blockSize, 0, stream>>>(depth.ptr<float>(), depth_out.ptr<short>(), screen_out.ptr<short2>(), depth.step1(), depth_out.step1(), params, pose, camera); break;
		case ViewPortMode::Stretch: screen_coord_kernel<ViewPortMode::Stretch, Projection::PERSPECTIVE><<<gridSize, blockSize, 0, stream>>>(depth.ptr<float>(), depth_out.ptr<short>(), screen_out.ptr<short2>(), depth.step1(), depth_out.step1(), params, pose, camera); break;
		}
	} else if (params.projection == Projection::EQUIRECTANGULAR) {
		switch (params.viewPortMode) {
		case ViewPortMode::Disabled: screen_coord_kernel<ViewPortMode::Disabled, Projection::EQUIRECTANGULAR><<<gridSize, blockSize, 0, stream>>>(depth.ptr<float>(), depth_out.ptr<short>(), screen_out.ptr<short2>(), depth.step1(), depth_out.step1(), params, pose, camera); break;
		case ViewPortMode::Clipping: screen_coord_kernel<ViewPortMode::Clipping, Projection::EQUIRECTANGULAR><<<gridSize, blockSize, 0, stream>>>(depth.ptr<float>(), depth_out.ptr<short>(), screen_out.ptr<short2>(), depth.step1(), depth_out.step1(), params, pose, camera); break;
		case ViewPortMode::Stretch: screen_coord_kernel<ViewPortMode::Stretch, Projection::EQUIRECTANGULAR><<<gridSize, blockSize, 0, stream>>>(depth.ptr<float>(), depth_out.ptr<short>(), screen_out.ptr<short2>(), depth.step1(), depth_out.step1(), params, pose, camera); break;
		}
	} else if (params.projection == Projection::ORTHOGRAPHIC) {
		switch (params.viewPortMode) {
		case ViewPortMode::Disabled: screen_coord_kernel<ViewPortMode::Disabled, Projection::ORTHOGRAPHIC><<<gridSize, blockSize, 0, stream>>>(depth.ptr<float>(), depth_out.ptr<short>(), screen_out.ptr<short2>(), depth.step1(), depth_out.step1(), params, pose, camera); break;
		case ViewPortMode::Clipping: screen_coord_kernel<ViewPortMode::Clipping, Projection::ORTHOGRAPHIC><<<gridSize, blockSize, 0, stream>>>(depth.ptr<float>(), depth_out.ptr<short>(), screen_out.ptr<short2>(), depth.step1(), depth_out.step1(), params, pose, camera); break;
		case ViewPortMode::Stretch: screen_coord_kernel<ViewPortMode::Stretch, Projection::ORTHOGRAPHIC><<<gridSize, blockSize, 0, stream>>>(depth.ptr<float>(), depth_out.ptr<short>(), screen_out.ptr<short2>(), depth.step1(), depth_out.step1(), params, pose, camera); break;
		}
	}
	cudaSafeCall( cudaGetLastError() );
}


// ==== Constant depth version =================================================

/*
 * Convert source screen position to output screen coordinates. Assumes a
 * constant depth of 1m instead of using a depth channel input.
 */
 __global__ void screen_coord_kernel(
	 	short* __restrict__ depth_out,
		short2* __restrict__ screen_out,
		int pitch4, int pitch2,
		Camera vcamera, float4x4 pose, Camera camera) {

	const int x = blockIdx.x*blockDim.x + threadIdx.x;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x >= 0 && y >= 0 && x < camera.width && y < camera.height) {
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

		screen_out[y*pitch4+x] = make_short2(screenPos.x, screenPos.y);
		depth_out[y*pitch2+x] = cudatl::float2fixed<10>(camPos.z);
	}
}

void ftl::cuda::screen_coord(cv::cuda::GpuMat &depth_out, cv::cuda::GpuMat &screen_out, const Parameters &params, const float4x4 &pose, const Camera &camera, cudaStream_t stream) {
	static constexpr int THREADS_X = 8;
	static constexpr int THREADS_Y = 8;

    const dim3 gridSize((camera.width + THREADS_X - 1)/THREADS_X, (camera.height + THREADS_Y - 1)/THREADS_Y);
	const dim3 blockSize(THREADS_X, THREADS_Y);

	depth_out.create(camera.height, camera.width, CV_16S);
	screen_out.create(camera.height, camera.width, CV_16SC2);

	screen_coord_kernel<<<gridSize, blockSize, 0, stream>>>(depth_out.ptr<short>(), screen_out.ptr<short2>(), screen_out.step1()/2, depth_out.step1(), params.camera, pose, camera);
	cudaSafeCall( cudaGetLastError() );
}
