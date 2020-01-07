#include <ftl/cuda_common.hpp>

namespace ftl {
namespace cuda {

void disparity_to_depth(const cv::cuda::GpuMat &disparity, cv::cuda::GpuMat &depth,
						const ftl::rgbd::Camera &c, cudaStream_t &stream);

void depth_to_disparity(cv::cuda::GpuMat &disparity, const cv::cuda::GpuMat &depth,
						const ftl::rgbd::Camera &c, cudaStream_t &stream);


void optflow_filter(cv::cuda::GpuMat &disp, const cv::cuda::GpuMat &optflow,
					cv::cuda::GpuMat &history, int n_max, float threshold,
					cv::cuda::Stream &stream);

}
}
