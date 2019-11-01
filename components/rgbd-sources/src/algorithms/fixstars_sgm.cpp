/* Copyright 2019 Nicolas Pope */

#include "fixstars_sgm.hpp"
#include <loguru.hpp>
#include <opencv2/cudastereo.hpp>

using ftl::algorithms::FixstarsSGM;
using cv::Mat;
using cv::cuda::GpuMat;
using ftl::codecs::Channel;
using ftl::rgbd::Format;

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

	updateBilateralFilter();

	on("use_filter", [this](const ftl::config::Event&) {
		updateBilateralFilter();
	});

	on("filter_radius", [this](const ftl::config::Event&) {
		updateBilateralFilter();
	});

	on("filter_iter", [this](const ftl::config::Event&) {
		updateBilateralFilter();
	});
	
#ifdef HAVE_OPTFLOW
	updateOFDisparityFilter();

	on("use_off", [this](const ftl::config::Event&) {
		updateOFDisparityFilter();
	});

	on("use_off", [this](const ftl::config::Event&) {
		updateOFDisparityFilter();
	});
#endif

	init(size_);

	on("uniqueness", [this](const ftl::config::Event&) {
		float uniqueness = value("uniqueness", uniqueness_);
		if ((uniqueness >= 0.0) && (uniqueness <= 1.0)) {
			uniqueness_ = uniqueness;
			updateParameters();
		}
		else {
			LOG(WARNING) << "invalid uniquness: " << uniqueness;
		}
	});

	on("P1", [this](const ftl::config::Event&) {
		int P1 = value("P1", P1_);
		if (P1 <= P2_) {
			P1_ = P1;
			updateParameters();
		}
		else {
			LOG(WARNING) << "invalid P1: " << P1 << ", (P1 <= P2), P2 is " << P2_;
		}
	});

	on("P2", [this](const ftl::config::Event&) {
		int P2 = value("P2", P2_);
		if (P2 >= P1_) {
			P2_ = P2;
			updateParameters();
		}
		else {
			LOG(WARNING) << "invalid P2: " << P2 << ", (P1 <= P2), P1 is " << P1_;
		}
	});
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

bool FixstarsSGM::updateParameters() {
	if (ssgm_ == nullptr) { return false; }
	return this->ssgm_->updateParameters(
		sgm::StereoSGM::Parameters(P1_, P2_, uniqueness_, true));
}

bool FixstarsSGM::updateBilateralFilter() {
	bool enable = value("use_filter", false);
	int radius = value("filter_radius", 25);
	int iter = value("filter_iter", 1);

	if (enable) {
		if (radius <= 0) {
			LOG(WARNING) << "filter_radius must be greater than 0";
			enable = false;
		}
		if (iter <= 0) {
			LOG(WARNING) << "filter_iter must be greater than 0";
			enable = false;
		}
	}
	
	if (enable) {
		filter_ = cv::cuda::createDisparityBilateralFilter(max_disp_ << 4, radius, iter);
		use_filter_ = true;
	}
	else {
		use_filter_ = false;
	}

	return use_filter_;
}

#ifdef HAVE_OPTFLOW
bool FixstarsSGM::updateOFDisparityFilter() {
	bool enable = value("use_off", false);
	int off_size = value("off_size", 9);
	double off_threshold = value("off_threshold", 0.9);

	if (enable) {
		if (off_size <= 0) {
			LOG(WARNING) << "bad off_size: " << off_size;
			enable = false;
		}

		if (off_threshold <= 0.0) {
			LOG(WARNING) << "bad off_threshold: " << off_threshold;
			enable = false;
		}
	}
	
	if (enable) {
		LOG(INFO) << "Optical flow filter, size: " << off_size << ", threshold: " << off_threshold;
		off_ = ftl::rgbd::OFDisparityFilter(size_, off_size, off_threshold);
		use_off_ = true;
	}
	else {
		use_off_ = false;
	}
	
	return use_off_;
}
#endif

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

	const auto &l = frame.get<GpuMat>(Channel::Left);
	const auto &r = frame.get<GpuMat>(Channel::Right);
	auto &disp = frame.create<GpuMat>(Channel::Disparity, Format<float>(l.size()));

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

	#ifdef HAVE_OPTFLOW
	if (use_off_) {	
		frame.upload(Channel::Flow, stream);
		stream.waitForCompletion();
		off_.filter(dispt_, frame.get<GpuMat>(Channel::Flow), stream);
	}
	#endif

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

}

void FixstarsSGM::setMask(Mat &mask) {
	return; // TODO(Nick) Not needed, but also code below does not work with new GPU pipeline
	CHECK(mask.type() == CV_8UC1) << "mask type must be CV_8U";
	if (!ssgm_) { init(size_); }
	mask_l_ = GpuMat(mask);
	ssgm_->setMask((uint8_t*)mask.data, mask.cols);
}