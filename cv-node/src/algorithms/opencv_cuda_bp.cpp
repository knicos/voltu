#include <ftl/algorithms/opencv_cuda_bp.hpp>

using ftl::algorithms::OpenCVCudaBP;
using namespace cv;

static ftl::Disparity::Register opencvcudabp("cuda_bp", OpenCVCudaBP::create);

OpenCVCudaBP::OpenCVCudaBP(nlohmann::json &config) : Disparity(config) {
	matcher_ = cuda::createStereoBeliefPropagation(max_disp_);
	
	// TODO Add filter
}

void OpenCVCudaBP::compute(const cv::Mat &l, const cv::Mat &r, cv::Mat &disp) {
	if (disp_.empty()) disp_ = cuda::GpuMat(l.size(), CV_8U);
	if (left_.empty()) left_ = cuda::GpuMat(l.size(), CV_8U);
	if (right_.empty()) right_ = cuda::GpuMat(l.size(), CV_8U);
	
	left_.upload(l);
	right_.upload(r);
	
	matcher_->compute(left_, right_, disp_);
	
	disp_.download(disp);
}

