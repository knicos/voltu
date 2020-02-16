#include <ftl/cuda_common.hpp>
#include <ftl/rgbd/camera.hpp>
#include <opencv2/core/cuda_stream_accessor.hpp>

__global__ void d2d_kernel(cv::cuda::PtrStepSz<float> disp, cv::cuda::PtrStepSz<float> depth,
		ftl::rgbd::Camera cam) {

	for (STRIDE_Y(v,disp.rows)) {
	for (STRIDE_X(u,disp.cols)) {
		float d = disp(v,u);
		depth(v,u) = (d == 0) ? 0.0f : ((cam.baseline*cam.fx) / (d - cam.doffs));
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

__global__ void d2drev_kernel(cv::cuda::PtrStepSz<float> disp, cv::cuda::PtrStepSz<float> depth,
	ftl::rgbd::Camera cam) {

for (STRIDE_Y(v,disp.rows)) {
for (STRIDE_X(u,disp.cols)) {
	float d = depth(v,u);
	float disparity = (d > cam.maxDepth || d < cam.minDepth) ? 0.0f : ((cam.baseline*cam.fx) / d) + cam.doffs;
	disp(v,u) = disparity;
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
