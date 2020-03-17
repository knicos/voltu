#include <ftl/cuda_common.hpp>
#include <ftl/rgbd/camera.hpp>
#include <opencv2/core/cuda_stream_accessor.hpp>
#include <ftl/operators/cuda/disparity.hpp>
#include <ftl/operators/cuda/mask.hpp>

#ifndef PINF
#define PINF __int_as_float(0x7f800000)
#endif

__global__ void d2d_kernel(cv::cuda::PtrStepSz<short> disp, cv::cuda::PtrStepSz<float> depth,
		ftl::rgbd::Camera cam) {

	for (STRIDE_Y(v,disp.rows)) {
	for (STRIDE_X(u,disp.cols)) {
		short d = disp(v,u);
		depth(v,u) = (d == 0) ? 0.0f : ((cam.baseline*cam.fx) / ((float(d)/16.0f) - cam.doffs));
	}
	}
}

namespace ftl {
namespace cuda {
	void disparity_to_depth(const cv::cuda::GpuMat &disparity, cv::cuda::GpuMat &depth,
				const ftl::rgbd::Camera &c, cudaStream_t &stream) {
		dim3 grid(1,1,1);
		dim3 threads(128, 1, 1);
		grid.x = cv::cuda::device::divUp(disparity.cols, 128);
		grid.y = cv::cuda::device::divUp(disparity.rows, 1);
		d2d_kernel<<<grid, threads, 0, stream>>>(
			disparity, depth, c);
		cudaSafeCall( cudaGetLastError() );
	}
}
}

//==============================================================================

__global__ void d2drev_kernel(cv::cuda::PtrStepSz<short> disp, cv::cuda::PtrStepSz<float> depth,
	ftl::rgbd::Camera cam) {

for (STRIDE_Y(v,disp.rows)) {
for (STRIDE_X(u,disp.cols)) {
	float d = depth(v,u);
	float disparity = (d > cam.maxDepth || d < cam.minDepth) ? 0.0f : ((cam.baseline*cam.fx) / d) + cam.doffs;
	disp(v,u) = short(disparity*16.0f);
}
}
}

namespace ftl {
namespace cuda {
void depth_to_disparity(cv::cuda::GpuMat &disparity, const cv::cuda::GpuMat &depth,
			const ftl::rgbd::Camera &c, cudaStream_t &stream) {
	dim3 grid(1,1,1);
	dim3 threads(128, 1, 1);
	grid.x = cv::cuda::device::divUp(disparity.cols, 128);
	grid.y = cv::cuda::device::divUp(disparity.rows, 1);
	d2drev_kernel<<<grid, threads, 0, stream>>>(
		disparity, depth, c);
	cudaSafeCall( cudaGetLastError() );
}
}
}

// =============================================================================

__global__ void remove_occ_kernel(cv::cuda::PtrStepSz<float> depth, cv::cuda::PtrStepSz<float> depthR,
	ftl::rgbd::Camera cam)
{
	for (STRIDE_Y(v,depth.rows)) {
	for (STRIDE_X(u,depth.cols)) {
		float d = depth(v,u);
		int disparity = int((d > cam.maxDepth || d < cam.minDepth) ? 0.0f : ((cam.baseline*cam.fx) / d) + cam.doffs);

		if (disparity > 0 && u-disparity > 0) {
			float dR = depthR(v,u-disparity);
			if (fabsf(d-dR) > 0.01f*d) {
				depth(v,u) = 0.0f;
			}
		}
	}
	}
}

void ftl::cuda::remove_occlusions(cv::cuda::GpuMat &depth, const cv::cuda::GpuMat &depthR,
			const ftl::rgbd::Camera &c, cudaStream_t stream) {
	dim3 grid(1,1,1);
	dim3 threads(128, 4, 1);
	grid.x = cv::cuda::device::divUp(depth.cols, 128);
	grid.y = cv::cuda::device::divUp(depth.rows, 4);
	remove_occ_kernel<<<grid, threads, 0, stream>>>(
		depth, depthR, c);
	cudaSafeCall( cudaGetLastError() );
}

__global__ void mask_occ_kernel(cv::cuda::PtrStepSz<float> depth,
	cv::cuda::PtrStepSz<float> depthR,
	cv::cuda::PtrStepSz<uchar> mask,
	ftl::rgbd::Camera cam)
{
	for (STRIDE_Y(v,depth.rows)) {
	for (STRIDE_X(u,depth.cols)) {
		float d = depth(v,u);
		int disparity = int((d > cam.maxDepth || d < cam.minDepth) ? 0.0f : ((cam.baseline*cam.fx) / d) + cam.doffs);

		if (disparity > 0 && u-disparity > 0) {
			float dR = depthR(v,u-disparity);
			if (fabsf(d-dR) > 0.01f*d) {
				mask(v,u) = mask(v,u) | ftl::cuda::Mask::kMask_Occlusion;
			}
		}
	}
	}
}

void ftl::cuda::mask_occlusions(const cv::cuda::GpuMat &depth, const cv::cuda::GpuMat &depthR,
			cv::cuda::GpuMat &mask,
			const ftl::rgbd::Camera &c, cudaStream_t stream) {
	dim3 grid(1,1,1);
	dim3 threads(128, 4, 1);
	grid.x = cv::cuda::device::divUp(depth.cols, 128);
	grid.y = cv::cuda::device::divUp(depth.rows, 4);
	mask_occ_kernel<<<grid, threads, 0, stream>>>(
		depth, depthR, mask, c);
	cudaSafeCall( cudaGetLastError() );
}
