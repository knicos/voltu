#include <ftl/cuda_common.hpp>

namespace ftl {
namespace cuda {

void disparity_to_depth(const cv::cuda::GpuMat &disparity, cv::cuda::GpuMat &depth,
						const ftl::rgbd::Camera &c, cudaStream_t &stream);

void depth_to_disparity(cv::cuda::GpuMat &disparity, const cv::cuda::GpuMat &depth,
						const ftl::rgbd::Camera &c, cudaStream_t &stream);

void remove_occlusions(cv::cuda::GpuMat &depth, const cv::cuda::GpuMat &depthR,
						const ftl::rgbd::Camera &c, cudaStream_t stream);

void mask_occlusions(const cv::cuda::GpuMat &depth,
		const cv::cuda::GpuMat &depthR,
		cv::cuda::GpuMat &mask,
		const ftl::rgbd::Camera &c, cudaStream_t stream);


void optflow_filter(cv::cuda::GpuMat &disp, const cv::cuda::GpuMat &optflow,
					cv::cuda::GpuMat &history, cv::cuda::GpuMat &support, int n_max, float threshold, bool fill,
					cv::cuda::Stream &stream);

}
}
