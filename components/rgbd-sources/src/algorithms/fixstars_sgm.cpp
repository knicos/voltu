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
	P1_ = value("P1", 10);
	P2_ = value("P2", 120);
	
	CHECK((uniqueness_ >= 0.0) && (uniqueness_ <= 1.0));
	CHECK(P1_ >= 0);
	CHECK(P2_ > P1_);

	use_filter_ = value("use_filter", false);
	if (use_filter_) {
		int radius = value("filter_radius", 25);
		int iter = value("filter_iter", 1);
		CHECK(radius > 0) << "filter_radius must be greater than 0";
		CHECK(iter > 0) << "filter_iter must be greater than 0";
		
		filter_ = cv::cuda::createDisparityBilateralFilter(max_disp_ << 4, radius, iter);
	}
}

void FixstarsSGM::init(const cv::Size size) {
	if (ssgm_) { delete ssgm_; }
	dispt_ = GpuMat(size, CV_16SC1);
	ssgm_ = new sgm::StereoSGM(	size.width, size.height, max_disp_, 8, 16,
								lbw_.step, dispt_.step / sizeof(short),
								sgm::EXECUTE_INOUT_CUDA2CUDA,
								sgm::StereoSGM::Parameters(P1_, P2_, uniqueness_, true)
	);
}

void FixstarsSGM::compute(const cv::cuda::GpuMat &l, const cv::cuda::GpuMat &r, cv::cuda::GpuMat &disp, cv::cuda::Stream &stream) {
	cv::cuda::cvtColor(l, lbw_, cv::COLOR_BGR2GRAY, 0, stream);
	cv::cuda::cvtColor(r, rbw_, cv::COLOR_BGR2GRAY, 0, stream);

	stream.waitForCompletion();
	if (!ssgm_) { init(l.size()); }

	//auto start = std::chrono::high_resolution_clock::now();
	ssgm_->execute(lbw_.data, rbw_.data, dispt_.data);
	//std::chrono::duration<double> elapsed =
	//		std::chrono::high_resolution_clock::now() - start;
	//LOG(INFO) << "CUDA sgm in " << elapsed.count() << "s";
	
	GpuMat left_pixels(dispt_, cv::Rect(0, 0, max_disp_, dispt_.rows));
	left_pixels.setTo(0);

	cv::cuda::threshold(dispt_, dispt_, 4096.0f, 0.0f, cv::THRESH_TOZERO_INV, stream);

	if (use_filter_) { filter_->apply(dispt_, l, dispt_, stream); }
	
	dispt_.convertTo(disp, CV_32F, 1.0f/16.0f, stream);
}

void FixstarsSGM::setMask(Mat &mask) {
	return; // TODO(Nick) Not needed, but also code below does not work with new GPU pipeline
	CHECK(mask.type() == CV_8UC1) << "mask type must be CV_8U";
	
	if (!ssgm_) { init(mask.size()); }
	
	mask_l_ = mask;
	ssgm_->setMask((uint8_t*) mask.data, mask.cols);
}