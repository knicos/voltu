/* Copyright 2019 Nicolas Pope */

#include "fixstars_sgm.hpp"
#include <loguru.hpp>
#include <opencv2/cudastereo.hpp>

using ftl::algorithms::FixstarsSGM;
using cv::Mat;
using cv::cuda::GpuMat;

//static ftl::Disparity::Register fixstarssgm("libsgm", FixstarsSGM::create);

FixstarsSGM::FixstarsSGM(nlohmann::json &config) : Disparity(config) {
	ssgm_ = nullptr;
	uniqueness_ = value("uniqueness", 0.95f);
	use_filter_ = value("use_filter", false);
	filter_ = cv::cuda::createDisparityBilateralFilter(max_disp_ << 4, value("filter_radius", 25), value("filter_iter", 1));
}

void FixstarsSGM::compute(const cv::cuda::GpuMat &l, const cv::cuda::GpuMat &r, cv::cuda::GpuMat &disp, cv::cuda::Stream &stream) {
	cv::cuda::cvtColor(l, lbw_, cv::COLOR_BGR2GRAY, 0, stream);
	cv::cuda::cvtColor(r, rbw_, cv::COLOR_BGR2GRAY, 0, stream);

	stream.waitForCompletion();
	if (!ssgm_) { // todo: move to constructor
		dispt_ = GpuMat(l.rows, l.cols, CV_16SC1);
		ssgm_ = new sgm::StereoSGM(l.cols, l.rows, max_disp_, 8, 16, lbw_.step, dispt_.step / sizeof(short),
			sgm::EXECUTE_INOUT_CUDA2CUDA, sgm::StereoSGM::Parameters(10,120,0.95f,true));
	}

	//auto start = std::chrono::high_resolution_clock::now();
	ssgm_->execute(lbw_.data, rbw_.data, dispt_.data);
	//std::chrono::duration<double> elapsed =
	//		std::chrono::high_resolution_clock::now() - start;
	//LOG(INFO) << "CUDA sgm in " << elapsed.count() << "s";
	
	// todo: fix libSGM (return float data or provide mask separately)
	// disparity values set to (256 << 5) in libSGM consistency check 
	//Mat bad_pixels = (disp == (256 << 5)); 
	
	//disp.setTo(0, bad_pixels, stream_);
	GpuMat left_pixels(dispt_, cv::Rect(0, 0, max_disp_, dispt_.rows));
	left_pixels.setTo(0);

	cv::cuda::threshold(dispt_, dispt_, 4096.0f, 0.0f, cv::THRESH_TOZERO_INV, stream);

	if (use_filter_) {
		// parameters need benchmarking, impact of image
		// quick tests show with parameters (max_disp_, 25, 3)
		// roughly 50% in disparity calculation and 50% in filter;
		filter_->apply(dispt_, l, dispt_, stream);
	}
	
	dispt_.convertTo(disp, CV_32F, 1.0f/16.0f, stream);
}

void FixstarsSGM::setMask(Mat &mask) {
	return; // TODO(Nick) Not needed, but also code below does not work with new GPU pipeline
	CHECK(mask.type() == CV_8UC1) << "mask type must be CV_8U";
	
	if (!ssgm_) { // todo: move to constructor
		ssgm_ = new sgm::StereoSGM(mask.cols, mask.rows, max_disp_, 8, 16,
			sgm::EXECUTE_INOUT_HOST2HOST,
			sgm::StereoSGM::Parameters(10,120,0.95f,true));
	}
	
	mask_l_ = mask;
	ssgm_->setMask((uint8_t*) mask.data, mask.cols);
}