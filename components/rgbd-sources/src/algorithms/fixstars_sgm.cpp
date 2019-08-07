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

	int width = value("width", 1280);
	int height = value("height", 720);
	
	size_ = cv::Size(width, height);
	CHECK((width >= 480) && (height >= 360));

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

	init(size_);
}

void FixstarsSGM::init(const cv::Size size) {
	if (ssgm_) { delete ssgm_; }
	lbw_ = GpuMat(size, CV_8UC1);
	rbw_ = GpuMat(size, CV_8UC1);
	dispt_ = GpuMat(size, CV_16SC1);

	ssgm_ = new sgm::StereoSGM(size.width, size.height, max_disp_, 8, 16,
		lbw_.step, dispt_.step / sizeof(short),
		sgm::EXECUTE_INOUT_CUDA2CUDA,
		sgm::StereoSGM::Parameters(P1_, P2_, uniqueness_, true)
	);
}

void FixstarsSGM::compute(const cv::cuda::GpuMat &l, const cv::cuda::GpuMat &r,
	cv::cuda::GpuMat &disp, cv::cuda::Stream &stream)
{
	if (l.size() != size_) {
		// re-use same buffer for l/r
		cv::cuda::resize(r, l_downscaled_, size_, 0.0, 0.0, cv::INTER_CUBIC, stream);
		cv::cuda::cvtColor(l_downscaled_, rbw_, cv::COLOR_BGR2GRAY, 0, stream);
		cv::cuda::resize(l, l_downscaled_, size_, 0.0, 0.0, cv::INTER_CUBIC, stream);
		cv::cuda::cvtColor(l_downscaled_, lbw_, cv::COLOR_BGR2GRAY, 0, stream);
	}
	else {
		cv::cuda::cvtColor(l, lbw_, cv::COLOR_BGR2GRAY, 0, stream);
		cv::cuda::cvtColor(r, rbw_, cv::COLOR_BGR2GRAY, 0, stream);
	}

	stream.waitForCompletion();

	ssgm_->execute(lbw_.data, rbw_.data, dispt_.data);

	GpuMat left_pixels(dispt_, cv::Rect(0, 0, max_disp_, dispt_.rows));
	left_pixels.setTo(0);
	cv::cuda::threshold(dispt_, dispt_, 4096.0f, 0.0f, cv::THRESH_TOZERO_INV, stream);

	// TODO: filter could be applied after upscaling (to the upscaled disparity image)
	if (use_filter_) {
		filter_->apply(dispt_,
			l.size() != dispt_.size() ? l_downscaled_ : l,
			dispt_,
			stream
		);
	}

	if (l.size() != size_) {
		cv::cuda::multiply(dispt_, (double)l.cols / (double)size_.width, dispt_);
		// invalid areas (bad values) have to be taken into account in interpolation
		cv::cuda::resize(dispt_, dispt_full_res_, l.size(), 0.0, 0.0, cv::INTER_NEAREST, stream);
	}
	else {
		dispt_full_res_ = dispt_;
	}

	dispt_full_res_.convertTo(disp, CV_32F, 1.0f / 16.0f, stream);
}

void FixstarsSGM::setMask(Mat &mask) {
	return; // TODO(Nick) Not needed, but also code below does not work with new GPU pipeline
	CHECK(mask.type() == CV_8UC1) << "mask type must be CV_8U";

	if (!ssgm_) { init(size_); }

	mask_l_ = mask;
	ssgm_->setMask((uint8_t*)mask.data, mask.cols);
}