#ifndef _FTL_ALGORITHMS_OPENCV_CUDA_BM_HPP_
#define _FTL_ALGORITHMS_OPENCV_CUDA_BM_HPP_

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/cudastereo.hpp>
#include <ftl/disparity.hpp>

namespace ftl {
namespace algorithms {
class OpenCVCudaBM : public ftl::Disparity {
	public:
	OpenCVCudaBM(nlohmann::json &config);
	
	void compute(const cv::Mat &l, const cv::Mat &r, cv::Mat &disp);

	static inline Disparity *create(nlohmann::json &config) { return new OpenCVCudaBM(config); }
	
	private:
	cv::Ptr<cv::cuda::StereoBM> matcher_;
	cv::Ptr<cv::cuda::DisparityBilateralFilter> filter_;
	cv::cuda::GpuMat disp_;
	cv::cuda::GpuMat filtered_;
	cv::cuda::GpuMat left_;
	cv::cuda::GpuMat right_;
};
};
};

#endif // _FTL_ALGORITHMS_OPENCV_CUDA_BM_HPP_

