#include "opencv_cuda_bm.hpp"

using ftl::algorithms::OpenCVCudaBM;
using namespace cv;

static ftl::Disparity::Register opencvcudabm("cuda_bm", OpenCVCudaBM::create);

OpenCVCudaBM::OpenCVCudaBM(nlohmann::json &config) : Disparity(config) {
	matcher_ = cuda::createStereoBM(max_disp_);
	
	// TODO Add filter
	filter_ = cv::cuda::createDisparityBilateralFilter(max_disp_, 5, 5);
}

void OpenCVCudaBM::compute(const cv::Mat &l, const cv::Mat &r, cv::Mat &disp) {
	if (disp_.empty()) disp_ = cuda::GpuMat(l.size(), CV_8U);
	if (filtered_.empty()) filtered_ = cuda::GpuMat(l.size(), CV_8U);
	if (left_.empty()) left_ = cuda::GpuMat(l.size(), CV_8U);
	if (right_.empty()) right_ = cuda::GpuMat(l.size(), CV_8U);
	
	left_.upload(l);
	right_.upload(r);
	
	matcher_->compute(left_, right_, disp_);
	filter_->apply(disp_, left_, filtered_);
	
	filtered_.download(disp);
}



