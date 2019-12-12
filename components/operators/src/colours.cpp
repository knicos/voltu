#include <ftl/operators/colours.hpp>

using ftl::operators::ColourChannels;
using ftl::codecs::Channel;

ColourChannels::ColourChannels(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {

}

ColourChannels::~ColourChannels() {

}

bool ColourChannels::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, ftl::rgbd::Source *s, cudaStream_t stream) {
	auto cvstream = cv::cuda::StreamAccessor::wrapStream(stream);

	auto &col = in.get<cv::cuda::GpuMat>(Channel::Colour);

	// Convert colour from BGR to BGRA if needed
	if (col.type() == CV_8UC3) {
		//cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream);
		// Convert to 4 channel colour
		temp_.create(col.size(), CV_8UC4);
		cv::cuda::swap(col, temp_);
		cv::cuda::cvtColor(temp_,col, cv::COLOR_BGR2BGRA, 0, cvstream);
	}

	//in.resetTexture(Channel::Colour);
	in.createTexture<uchar4>(Channel::Colour, true);

	auto &depth = in.get<cv::cuda::GpuMat>(Channel::Depth);
	if (depth.size() != col.size()) {
		auto &col2 = in.create<cv::cuda::GpuMat>(Channel::ColourHighRes);
		cv::cuda::resize(col, col2, depth.size(), 0.0, 0.0, cv::INTER_LINEAR, cvstream);
		in.createTexture<uchar4>(Channel::ColourHighRes, true);
		in.swapChannels(Channel::Colour, Channel::ColourHighRes);
	}

	return true;
}
