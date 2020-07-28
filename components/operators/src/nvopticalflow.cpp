#include <ftl/operators/opticalflow.hpp>
#include <ftl/exception.hpp>
#include <ftl/operators/cuda/disparity.hpp>

#include "opticalflow_cuda.hpp"

#include <opencv2/cudaimgproc.hpp>

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

using ftl::rgbd::Frame;
using ftl::rgbd::Source;
using ftl::codecs::Channel;

using ftl::operators::NVOpticalFlow;

using cv::Size;
using cv::cuda::GpuMat;

NVOpticalFlow::NVOpticalFlow(ftl::Configurable* cfg) :
		ftl::operators::Operator(cfg), channel_in_{ftl::codecs::Channel::Colour,ftl::codecs::Channel::Colour2}, channel_out_{ftl::codecs::Channel::Flow,ftl::codecs::Channel::Flow2} {
	size_ = Size(0, 0);

}

NVOpticalFlow::NVOpticalFlow(ftl::Configurable*cfg, const std::tuple<ftl::codecs::Channel,ftl::codecs::Channel,ftl::codecs::Channel,ftl::codecs::Channel> &channels) : ftl::operators::Operator(cfg) {
	channel_in_[0] = std::get<0>(channels);
	channel_out_[0] = std::get<1>(channels);
	channel_in_[1] = std::get<2>(channels);
	channel_out_[1] = std::get<3>(channels);
	size_ = Size(0, 0);
}

NVOpticalFlow::~NVOpticalFlow() {
}

bool NVOpticalFlow::init() {
	if (!ftl::cuda::hasCompute(7,5)) {
		config()->set("enabled", false);
		//throw FTL_Error("GPU does not support optical flow");
		LOG(ERROR) << "GPU does not support optical flow";
		return false;
	}
	nvof_ = cv::cuda::NvidiaOpticalFlow_1_0::create(
				size_.width, size_.height, 
				cv::cuda::NvidiaOpticalFlow_1_0::NV_OF_PERF_LEVEL_SLOW,
				true, false, false, 0);
	
	left_gray_.create(size_, CV_8UC1);
	left_gray_prev_.create(size_, CV_8UC1);
	right_gray_.create(size_, CV_8UC1);
	right_gray_prev_.create(size_, CV_8UC1);
	return true;
}

bool NVOpticalFlow::apply(Frame &in, Frame &out, cudaStream_t stream) {
	bool both_channels = config()->value("both_channels", false);
	float scale = config()->value("viz_scale", 200.0f);

	if (!in.hasChannel(channel_in_[0])) return false;
	if (in.hasChannel(channel_out_[0])) return true;
	if (both_channels) {
		if (!in.hasChannel(channel_in_[1])) return false;
		if (in.hasChannel(channel_out_[1])) return true;
	}

	if (in.get<GpuMat>(channel_in_[0]).size() != size_) {
		size_ = in.get<GpuMat>(channel_in_[0]).size();
		if (!init()) return false;
	}
	
	auto cvstream = cv::cuda::StreamAccessor::wrapStream(stream);
	auto &flow1 = out.create<GpuMat>(channel_out_[0]);

	cv::cuda::cvtColor(in.get<GpuMat>(channel_in_[0]), left_gray_, cv::COLOR_BGRA2GRAY, 0, cvstream);
	if (both_channels) cv::cuda::cvtColor(in.get<GpuMat>(channel_in_[1]), right_gray_, cv::COLOR_BGRA2GRAY, 0, cvstream);

	// TODO: Use optical flow confidence output, perhaps combined with a
	// sensitivity adjustment
	nvof_->calc(left_gray_, right_gray_, flow1, cvstream);
	//std::swap(left_gray_, left_gray_prev_);

	if (both_channels) {
		auto &flow2 = out.create<GpuMat>(channel_out_[1]);
		nvof_->calc(right_gray_, left_gray_, flow2, cvstream);
		//std::swap(right_gray_, right_gray_prev_);
	}

	if (config()->value("show_flow", false)) {
		ftl::cuda::show_optical_flow(flow1, in.getTexture<uchar4>(channel_in_[0]), scale, stream);
		if (both_channels)
			ftl::cuda::show_optical_flow(out.get<GpuMat>(channel_out_[1]), in.getTexture<uchar4>(channel_in_[1]), scale, stream);
	}

	if (config()->value("generate_disparity", false)) {
		if (!in.hasChannel(Channel::Disparity)) {
			in.create<GpuMat>(Channel::Disparity).create(size_, CV_16SC1);
		}
		ftl::cuda::disparity_from_flow(
			flow1,
			out.get<GpuMat>(channel_out_[1]),
			in.createTexture<short>(Channel::Disparity), stream);
	}

	if (config()->value("check_reprojection", false)) {
		ftl::cuda::check_reprojection(in.get<cv::cuda::GpuMat>(Channel::Disparity), in.getTexture<uchar4>(Channel::Colour),
			in.createTexture<uchar4>(Channel::Colour2, true),
			stream);
	}

	return true;
}