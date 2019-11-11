#include <ftl/operators/colours.hpp>

using ftl::operators::ColourChannels;
using ftl::codecs::Channel;

ColourChannels::ColourChannels(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {

}

ColourChannels::~ColourChannels() {

}

bool ColourChannels::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, ftl::rgbd::Source *s, cudaStream_t stream) {
	auto cvstream = cv::cuda::StreamAccessor::wrapStream(stream);

	// Convert colour from BGR to BGRA if needed
	if (in.get<cv::cuda::GpuMat>(Channel::Colour).type() == CV_8UC3) {
		//cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream);
		// Convert to 4 channel colour
		auto &col = in.get<cv::cuda::GpuMat>(Channel::Colour);
		temp_.create(col.size(), CV_8UC4);
		cv::cuda::swap(col, temp_);
		cv::cuda::cvtColor(temp_,col, cv::COLOR_BGR2BGRA, 0, cvstream);
	}

	return true;
}
