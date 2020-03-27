#include <ftl/cuda_common.hpp>

namespace ftl {
namespace cuda {

template<typename T_in, typename T_out>
void disparity_to_depth(const cv::cuda::GpuMat &depth, cv::cuda::GpuMat &disparity,
						const ftl::rgbd::Camera &c, float scale, cudaStream_t &stream);

template<typename T_in, typename T_out>
void depth_to_disparity(const cv::cuda::GpuMat &depth, cv::cuda::GpuMat &disparity,
						const ftl::rgbd::Camera &c, float scale, cudaStream_t &stream);

void remove_occlusions(cv::cuda::GpuMat &depth, const cv::cuda::GpuMat &depthR,
						const ftl::rgbd::Camera &c, cudaStream_t stream);

void mask_occlusions(const cv::cuda::GpuMat &depth,
		const cv::cuda::GpuMat &depthR,
		cv::cuda::GpuMat &mask,
		const ftl::rgbd::Camera &c, cudaStream_t stream);

void check_reprojection(const cv::cuda::GpuMat &disp,
			const ftl::cuda::TextureObject<uchar4> &left, const ftl::cuda::TextureObject<uchar4> &right,
			cudaStream_t stream);

void show_rpe(const cv::cuda::GpuMat &disp, cv::cuda::GpuMat &left, const cv::cuda::GpuMat &right,
			float scale, cudaStream_t stream);

void show_disp_density(const cv::cuda::GpuMat &disp, cv::cuda::GpuMat &left,
			float scale, cudaStream_t stream);

void merge_disparities(cv::cuda::GpuMat &disp, const cv::cuda::GpuMat &estimate, cudaStream_t stream);


void optflow_filter(cv::cuda::GpuMat &disp, const cv::cuda::GpuMat &optflow,
					cv::cuda::GpuMat &history, cv::cuda::GpuMat &support, int n_max, float threshold, bool fill,
					cv::cuda::Stream &stream);

}
}
