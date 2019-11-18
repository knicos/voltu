#include <ftl/operators/opticalflow.hpp>

using ftl::rgbd::Frame;
using ftl::rgbd::Source;
using ftl::codecs::Channel;

using ftl::operators::NVOpticalFlow;

using cv::Size;
using cv::cuda::GpuMat;

NVOpticalFlow::NVOpticalFlow(ftl::Configurable* cfg) :
		ftl::operators::Operator(cfg) {
	size_ = Size(0, 0);
}

NVOpticalFlow::~NVOpticalFlow() {
}

bool NVOpticalFlow::init() {
	nvof_ = cv::cuda::NvidiaOpticalFlow_1_0::create(
				size_.width, size_.height, 
				cv::cuda::NvidiaOpticalFlow_1_0::NV_OF_PERF_LEVEL_SLOW,
				true, false, false, 0);
	
	left_gray_.create(size_, CV_8UC1);
	left_gray_prev_.create(size_, CV_8UC1);
	return true;
}

bool NVOpticalFlow::apply(Frame &in, Frame &out, Source *src, cudaStream_t stream) {
	if (!in.hasChannel(channel_in_)) { return false; }

	if (in.get<GpuMat>(channel_in_).size() != size_) {
		size_ = in.get<GpuMat>(channel_in_).size();
		init();
	}
	
	auto cvstream = cv::cuda::StreamAccessor::wrapStream(stream);
	auto &flow = out.create<GpuMat>(channel_out_);

	cv::cuda::cvtColor(in.get<GpuMat>(channel_in_), left_gray_, cv::COLOR_BGR2GRAY, 0, cvstream);

	nvof_->calc(left_gray_, left_gray_prev_, flow, cvstream);
	std::swap(left_gray_, left_gray_prev_);

	return true;
}