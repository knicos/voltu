#include <ftl/operators/colours.hpp>

#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>

using ftl::operators::ColourChannels;
using ftl::codecs::Channel;

ColourChannels::ColourChannels(ftl::operators::Graph *g, ftl::Configurable *cfg) : ftl::operators::Operator(g, cfg) {

}

ColourChannels::~ColourChannels() {

}

bool ColourChannels::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) {
	if (!in.hasChannel(Channel::Colour)) {
		in.message(ftl::data::Message::Warning_MISSING_CHANNEL, "No colour channel found");
		return false;
	}

	auto &col = in.get<cv::cuda::GpuMat>(Channel::Colour);

	// Convert colour from BGR to BGRA if needed
	if (col.type() == CV_8UC3) {
		//cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream);
		// Convert to 4 channel colour
		/*temp_.create(col.size(), CV_8UC4);
		cv::cuda::swap(col, temp_);
		cv::cuda::cvtColor(temp_,col, cv::COLOR_BGR2BGRA, 0, cvstream);*/

		in.message(ftl::data::Message::Error_BAD_FORMAT, "Bad colour format");
		throw FTL_Error("Left colour must be 4 channels");
	}

	if (in.hasChannel(Channel::Right)) {
		auto &col = in.get<cv::cuda::GpuMat>(Channel::Right);

		// Convert colour from BGR to BGRA if needed
		if (col.type() == CV_8UC3) {
			//cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream);
			// Convert to 4 channel colour
			/*temp_.create(col.size(), CV_8UC4);
			cv::cuda::swap(col, temp_);
			cv::cuda::cvtColor(temp_,col, cv::COLOR_BGR2BGRA, 0, cvstream);*/

			in.message(ftl::data::Message::Error_BAD_FORMAT, "Bad colour format");
			throw FTL_Error("Right colour must be 4 channels");
		}
	}

	//in.resetTexture(Channel::Colour);
	const auto &vf = in.get<ftl::rgbd::VideoFrame>(Channel::Colour);
	if (vf.isGPU()) {
		in.createTexture<uchar4>(Channel::Colour, true);
	}

	if (in.hasChannel(Channel::Right)) {
		const auto &vf = in.get<ftl::rgbd::VideoFrame>(Channel::Right);
		if (vf.isGPU()) {
			in.createTexture<uchar4>(Channel::Right, true);
		}
	}

	/*if (in.hasChannel(Channel::Depth)) {
		auto &depth = in.get<cv::cuda::GpuMat>(Channel::Depth);
		if (depth.size() != col.size()) {
			auto cvstream = cv::cuda::StreamAccessor::wrapStream(stream);
			auto &col2 = in.create<cv::cuda::GpuMat>(Channel::ColourHighRes);
			cv::cuda::resize(col, col2, depth.size(), 0.0, 0.0, cv::INTER_LINEAR, cvstream);
			in.createTexture<uchar4>(Channel::ColourHighRes, true);
			in.swapChannels(Channel::Colour, Channel::ColourHighRes);

			//throw FTL_Error("Depth and colour channels and different resolutions: " << depth.rows << " vs " << col.rows);
		}

		// Ensure right channel is also downsized
		if (in.hasChannel(Channel::Right)) {
			auto &right = in.get<cv::cuda::GpuMat>(Channel::Right);
			if (depth.size() != right.size()) {
				//cv::cuda::resize(right, rbuf_, depth.size(), 0.0, 0.0, cv::INTER_LINEAR, cvstream);
				//cv::cuda::swap(right, rbuf_);

				throw FTL_Error("Depth and colour channels and different resolutions: " << depth.size() << " vs " << right.size());
			}
		}
	}*/

	return true;
}
