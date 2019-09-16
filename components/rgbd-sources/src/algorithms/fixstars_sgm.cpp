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
	const int width = size_.width;
	const int height = size_.height;

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
	
#ifdef HAVE_OPTFLOW
	use_off_ = value("use_off", false);

	if (use_off_)
	{
		int off_size = value("off_size", 9);
		double off_threshold = value("off_threshold", 0.9);
		off_ = ftl::rgbd::OFDisparityFilter(size_, off_size, off_threshold);
	}
#endif

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

void FixstarsSGM::compute(ftl::rgbd::Frame &frame, cv::cuda::Stream &stream)
{
	/*if (!frame.hasChannel(ftl::rgbd::kChanLeftGray))
	{
		auto &rgb = frame.getChannel<GpuMat>(ftl::rgbd::kChanLeft, stream);
		auto &gray = frame.setChannel<GpuMat>(ftl::rgbd::kChanLeftGray);
		cv::cuda::cvtColor(rgb, gray, cv::COLOR_BGR2GRAY, 0, stream);
	}

	if (!frame.hasChannel(ftl::rgbd::kChanRightGray))
	{
		auto &rgb = frame.getChannel<GpuMat>(ftl::rgbd::kChanRight, stream);
		auto &gray = frame.setChannel<GpuMat>(ftl::rgbd::kChanRightGray);
		cv::cuda::cvtColor(rgb, gray, cv::COLOR_BGR2GRAY, 0, stream);
	}*/

	const auto &l = frame.getChannel<GpuMat>(ftl::rgbd::kChanLeft, stream);
	const auto &r = frame.getChannel<GpuMat>(ftl::rgbd::kChanRight, stream);
	auto &disp = frame.setChannel<GpuMat>(ftl::rgbd::kChanDisparity);

	if (disp.size() != l.size())
	{
		disp = GpuMat(l.size(), CV_32FC1);
	}

	GpuMat l_scaled;
	if (l.size() != size_)
	{
		GpuMat _r;
		scaleInput(l, r, l_scaled, _r, stream);
		cv::cuda::cvtColor(l_scaled, lbw_, cv::COLOR_BGR2GRAY, 0, stream);
		cv::cuda::cvtColor(_r, rbw_, cv::COLOR_BGR2GRAY, 0, stream);
	}
	else
	{
		cv::cuda::cvtColor(l, lbw_, cv::COLOR_BGR2GRAY, 0, stream);
		cv::cuda::cvtColor(r, rbw_, cv::COLOR_BGR2GRAY, 0, stream);
	}

	stream.waitForCompletion();
	ssgm_->execute(lbw_.data, rbw_.data, dispt_.data);
	GpuMat left_pixels(dispt_, cv::Rect(0, 0, max_disp_, dispt_.rows));
	left_pixels.setTo(0);
	cv::cuda::threshold(dispt_, dispt_, 4096.0f, 0.0f, cv::THRESH_TOZERO_INV, stream);

	// TODO: filter could be applied after upscaling (to the upscaled disparity image)
	if (use_filter_)
	{
		filter_->apply(
			dispt_,
			(l.size() == size_) ? l : l_scaled,
			dispt_,
			stream
		);
	}

	GpuMat dispt_scaled;
	if (l.size() != size_)
	{
		scaleDisparity(l.size(), dispt_, dispt_scaled, stream);
	}
	else
	{
		dispt_scaled = dispt_;
	}

	dispt_scaled.convertTo(disp, CV_32F, 1.0f / 16.0f, stream);

#ifdef HAVE_OPTFLOW
	if (use_off_) { off_.filter(frame, stream); }
#endif
}

void FixstarsSGM::setMask(Mat &mask) {
	return; // TODO(Nick) Not needed, but also code below does not work with new GPU pipeline
	CHECK(mask.type() == CV_8UC1) << "mask type must be CV_8U";
	if (!ssgm_) { init(size_); }
	mask_l_ = GpuMat(mask);
	ssgm_->setMask((uint8_t*)mask.data, mask.cols);
}