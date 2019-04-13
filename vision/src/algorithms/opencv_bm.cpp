#include <ftl/algorithms/opencv_bm.hpp>

using ftl::algorithms::OpenCVBM;
using namespace cv::ximgproc;
using namespace cv;

static ftl::Disparity::Register opencvbm("bm", OpenCVBM::create);

OpenCVBM::OpenCVBM(nlohmann::json &config) : Disparity(config) {
	int wsize = config.value("windows_size", 5);
	float sigma = config.value("sigma", 1.5);
	float lambda = config.value("lambda", 8000.0);

	left_matcher_  = StereoBM::create(max_disp_,wsize);
    wls_filter_ = createDisparityWLSFilter(left_matcher_);
    right_matcher_ = createRightMatcher(left_matcher_);
    
    wls_filter_->setLambda(lambda);
    wls_filter_->setSigmaColor(sigma);
}

void OpenCVBM::compute(const cv::Mat &l, const cv::Mat &r, cv::Mat &disp) {
	Mat left_disp;
	Mat right_disp;
    Mat lg, rg;
	cv::cvtColor(l, lg, cv::COLOR_BGR2GRAY);
	cv::cvtColor(r, rg, cv::COLOR_BGR2GRAY);
	left_matcher_-> compute(lg, rg,left_disp);
    right_matcher_->compute(rg, lg, right_disp);
    wls_filter_->filter(left_disp, l, disp, right_disp);
}



