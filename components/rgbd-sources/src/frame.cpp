
#include <ftl/rgbd/frame.hpp>

namespace ftl {
namespace rgbd {

template<> const cv::Mat& Frame::getChannel(const ftl::rgbd::channel_t& channel, cv::cuda::Stream &stream)
{
	size_t idx = _channelIdx(channel);
	if (!(available_[idx] & mask_host))
	{
		if (available_[idx] & mask_gpu)
		{
			channels_gpu_[idx].download(channels_host_[idx], stream);
			available_[idx] |= mask_host;
		}
	}

	return channels_host_[idx];
}

template<> const cv::Mat& Frame::getChannel(const ftl::rgbd::channel_t& channel)
{
	auto &stream = cv::cuda::Stream::Null();
	auto &retval = getChannel<cv::Mat>(channel, stream);
	stream.waitForCompletion();
	return retval;
}

template<> cv::Mat& Frame::setChannel(const ftl::rgbd::channel_t& channel)
{
	size_t idx = _channelIdx(channel);
	available_[idx] = mask_host;
	return channels_host_[idx];
}

template<> const cv::cuda::GpuMat& Frame::getChannel(const ftl::rgbd::channel_t& channel, cv::cuda::Stream &stream)
{
	size_t idx = _channelIdx(channel);
	if (!(available_[idx] & mask_gpu))
	{
		if (available_[idx] & mask_host)
		{
			channels_gpu_[idx].upload(channels_host_[idx], stream);
			available_[idx] |= mask_gpu;
		}
	}
	
	return channels_gpu_[idx];
}

template<> const cv::cuda::GpuMat& Frame::getChannel(const ftl::rgbd::channel_t& channel)
{
	auto &stream = cv::cuda::Stream::Null();
	auto &retval = getChannel<cv::cuda::GpuMat>(channel, stream);
	stream.waitForCompletion();
	return retval;
}

template<> cv::cuda::GpuMat& Frame::setChannel(const ftl::rgbd::channel_t& channel)
{
	size_t idx = _channelIdx(channel);
	available_[idx] = mask_gpu;
	return channels_gpu_[idx];
}

}
}