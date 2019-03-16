#include <ftl/algorithms/opencv_sgbm.hpp>

using ftl::algorithms::OpenCVSGBM;
using namespace cv::ximgproc;
using namespace cv;

static ftl::Disparity::Register opencvsgbm("sgbm", OpenCVSGBM::create);

OpenCVSGBM::OpenCVSGBM(nlohmann::json &config) : Disparity(config) {
	int wsize = 5;
	float sigma = 1.5;
	float lambda = 8000.0;
	

	left_matcher_  = StereoSGBM::create(min_disp_,max_disp_,wsize);
            left_matcher_->setP1(24*wsize*wsize);
            left_matcher_->setP2(96*wsize*wsize);
            left_matcher_->setPreFilterCap(63);
            left_matcher_->setMode(StereoSGBM::MODE_SGBM_3WAY);
    wls_filter_ = createDisparityWLSFilter(left_matcher_);
    right_matcher_ = createRightMatcher(left_matcher_);
    
    wls_filter_->setLambda(lambda);
    wls_filter_->setSigmaColor(sigma);
}

void OpenCVSGBM::compute(const cv::Mat &l, const cv::Mat &r, cv::Mat &disp) {
	Mat left_disp;
	Mat right_disp;
	left_matcher_-> compute(l, r,left_disp);
    right_matcher_->compute(r,l, right_disp);
    wls_filter_->filter(left_disp,l,disp,right_disp);
}



