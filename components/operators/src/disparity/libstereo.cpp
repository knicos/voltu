#include <loguru.hpp>

#include "ftl/operators/disparity.hpp"
#include <ftl/operators/cuda/disparity.hpp>

#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaarithm.hpp>

#include <stereo.hpp>

using cv::Size;
using cv::cuda::GpuMat;

using ftl::rgbd::Format;
using ftl::codecs::Channel;
using ftl::rgbd::Frame;
using ftl::rgbd::Source;
using ftl::operators::StereoDisparity;

struct StereoDisparity::Impl {
	StereoCensusSgm sgm;
};

StereoDisparity::StereoDisparity(ftl::operators::Graph *g, ftl::Configurable* cfg) :
		ftl::operators::Operator(g, cfg), impl_(nullptr) {

	init();
}

StereoDisparity::~StereoDisparity() {
	if (impl_) {
		delete impl_;
		impl_ = nullptr;
	}
}

bool StereoDisparity::init() {
	if (impl_) { delete impl_; }
	impl_ = new Impl();
	impl_->sgm.params.d_min = 0;
	impl_->sgm.params.d_max = 256;
	return true;
}

bool StereoDisparity::apply(Frame &in, Frame &out, cudaStream_t stream) {
	if (!in.hasChannel(Channel::Left) || !in.hasChannel(Channel::Right)) {
		LOG(ERROR) << "Left or Right channel missing for stereo disparity";
		return false;
	}

	impl_->sgm.params.P1 = config()->value("P1", 10);
	impl_->sgm.params.P2 = config()->value("P2", 60);

	const auto &l = in.get<GpuMat>(Channel::Left);
	const auto &r = in.get<GpuMat>(Channel::Right);
	disp32f_.create(l.size(), CV_32FC1);

	bool has_estimate = in.hasChannel(Channel::Disparity);
	auto &disp = (!has_estimate) ? out.create<ftl::rgbd::VideoFrame>(Channel::Disparity).createGPU(Format<short>(l.size())) : in.get<GpuMat>(Channel::Disparity);

	auto cvstream = cv::cuda::StreamAccessor::wrapStream(stream);

	cvstream.waitForCompletion();
	impl_->sgm.compute(l, r, disp32f_);

	disp32f_.convertTo(disp, CV_16SC1, 16.0, 0.0, cvstream);
	return true;
}
