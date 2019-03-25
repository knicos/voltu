#include <ftl/algorithms/opencv_sgbm.hpp>

using ftl::algorithms::OpenCVSGBM;
using namespace cv::ximgproc;
using namespace cv;

static ftl::Disparity::Register opencvsgbm("sgbm", OpenCVSGBM::create);

OpenCVSGBM::OpenCVSGBM(nlohmann::json &config) : Disparity(config) {
	int wsize = config.value("windows_size", 5);
	float sigma = config.value("sigma", 1.5);
	float lambda = config.value("lambda", 8000.0);

	left_matcher_  = StereoSGBM::create(min_disp_,max_disp_,wsize);
            left_matcher_->setP1(24*wsize*wsize);
            left_matcher_->setP2(96*wsize*wsize);
            left_matcher_->setPreFilterCap(63);
            left_matcher_->setMode(StereoSGBM::MODE_SGBM_3WAY);
            left_matcher_->setMinDisparity(config.value("minimum",0));
    wls_filter_ = createDisparityWLSFilter(left_matcher_);
    right_matcher_ = createRightMatcher(left_matcher_);
    
    wls_filter_->setLambda(lambda);
    wls_filter_->setSigmaColor(sigma);
}

void OpenCVSGBM::compute(const cv::Mat &l, const cv::Mat &r, cv::Mat &disp) {
	Mat lbw, rbw;
	Mat left_disp;
	Mat right_disp;
	cv::cvtColor(l, lbw,  COLOR_BGR2GRAY);
	cv::cvtColor(r, rbw, COLOR_BGR2GRAY);
	left_matcher_-> compute(lbw, rbw,left_disp);
    right_matcher_->compute(rbw,lbw, right_disp);
    wls_filter_->filter(left_disp,l,disp,right_disp);
    
    // WHY 12!!!!!!
    disp.convertTo(disp, CV_32F, 12.0 / max_disp_);
}



