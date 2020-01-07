#include <loguru.hpp>

#include "ftl/operators/disparity.hpp"

using cv::Size;
using cv::cuda::GpuMat;

using ftl::rgbd::Format;
using ftl::codecs::Channel;
using ftl::rgbd::Frame;
using ftl::rgbd::Source;
using ftl::operators::FixstarsSGM;

FixstarsSGM::FixstarsSGM(ftl::Configurable* cfg) :
		ftl::operators::Operator(cfg) {
	
	ssgm_ = nullptr;
	size_ = Size(0, 0);

	uniqueness_ = cfg->value("uniqueness", 0.95f);
	P1_ = cfg->value("P1", 10);
	P2_ = cfg->value("P2", 120);
	max_disp_ = cfg->value("max_disp", 256);

	if (uniqueness_ < 0.0 || uniqueness_ > 1.0) {
		uniqueness_ = 1.0;
		LOG(WARNING) << "Invalid uniqueness, using default (1.0)";
	}

	if (P1_ <= 0 ) {
		P1_ = 10;
		LOG(WARNING) << "Invalid value for P1, using default (10)";
	}

	if (P2_ < P1_) {
		P2_ = P1_;
		LOG(WARNING) << "Invalid value for P2, using value of P1 (" << P1_ << ")";
	}

	if (!(max_disp_ == 256 || max_disp_ == 128)) {
		max_disp_ = 256;
		LOG(WARNING) << "Invalid value for max_disp, using default value (256)";
	}

	cfg->on("P1", [this, cfg](const ftl::config::Event&) {
		int P1 = cfg->value("P1", 0);
		if (P1 <= 0) {
			LOG(WARNING) << "Invalid value for P1 (" << P1 << ")";
		}
		else {
			P1_ = P1;
			updateParameters();
		}
	});

	cfg->on("P2", [this, cfg](const ftl::config::Event&) {
		int P2 = cfg->value("P2", 0);
		if (P2 < P1_) {
			LOG(WARNING) << "Invalid value for P2 (" << P2 << ")";
		}
		else {
			P2_ = P2;
			updateParameters();
		}
	});

	cfg->on("uniqueness", [this, cfg](const ftl::config::Event&) {
		double uniqueness = cfg->value("uniqueness", 0.0);
		if (uniqueness < 0.0 || uniqueness > 1.0) {
			LOG(WARNING) << "Invalid value for uniqueness (" << uniqueness << ")";
		}
		else {
			uniqueness_ = uniqueness;
			updateParameters();
		}
	});
}

FixstarsSGM::~FixstarsSGM() {
	if (ssgm_) {
		delete ssgm_;
	}
}

bool FixstarsSGM::init() {
	if (size_ == Size(0, 0)) { return false; }
	if (ssgm_) { delete ssgm_; }
	lbw_.create(size_, CV_8UC1);
	rbw_.create(size_, CV_8UC1);
	disp_int_.create(size_, CV_16SC1);

	LOG(INFO) << "INIT FIXSTARS";

	ssgm_ = new sgm::StereoSGM(size_.width, size_.height, max_disp_, 8, 16,
		lbw_.step, disp_int_.step / sizeof(short),
		sgm::EXECUTE_INOUT_CUDA2CUDA,
		sgm::StereoSGM::Parameters(P1_, P2_, uniqueness_, true)
	);

	return true;
}

bool FixstarsSGM::updateParameters() {
	if (ssgm_ == nullptr) { return false; }
	return this->ssgm_->updateParameters(
		sgm::StereoSGM::Parameters(P1_, P2_, uniqueness_, true));
}

bool FixstarsSGM::apply(Frame &in, Frame &out, Source *src, cudaStream_t stream) {
	if (!in.hasChannel(Channel::Left) || !in.hasChannel(Channel::Right)) {
		LOG(ERROR) << "Fixstars is missing Left or Right channel";
		return false;
	}

	const auto &l = in.get<GpuMat>(Channel::Left);
	const auto &r = in.get<GpuMat>(Channel::Right);

	if (l.size() != size_) {
		size_ = l.size();
		if (!init()) { return false; }
	}

	auto &disp = out.create<GpuMat>(Channel::Disparity, Format<float>(l.size()));

	auto cvstream = cv::cuda::StreamAccessor::wrapStream(stream);
	cv::cuda::cvtColor(l, lbw_, cv::COLOR_BGRA2GRAY, 0, cvstream);
	cv::cuda::cvtColor(r, rbw_, cv::COLOR_BGRA2GRAY, 0, cvstream);

	cvstream.waitForCompletion();
	ssgm_->execute(lbw_.data, rbw_.data, disp_int_.data);

	// GpuMat left_pixels(dispt_, cv::Rect(0, 0, max_disp_, dispt_.rows));
	// left_pixels.setTo(0);

	cv::cuda::threshold(disp_int_, disp_int_, 4096.0f, 0.0f, cv::THRESH_TOZERO_INV, cvstream);
	
	disp_int_.convertTo(disp, CV_32F, 1.0f / 16.0f, cvstream);
	return true;
}
