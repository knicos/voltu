#pragma once
#ifndef _FTL_RGBD_FRAME_HPP_
#define _FTL_RGBD_FRAME_HPP_

#include <ftl/configuration.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>

namespace ftl {
namespace rgbd {

typedef unsigned int channel_t;

static const channel_t kChanNone = 0;
static const channel_t kChanColour = 0x0001;
static const channel_t kChanLeft = 0x0001;		// CV_8UC3
static const channel_t kChanDepth = 0x0002;		// CV_32FC1
static const channel_t kChanRight = 0x0004;		// CV_8UC3
static const channel_t kChanDisparity = 0x0008; // CV_32FC1
static const channel_t kChanDeviation = 0x0010;
static const channel_t kChanNormals = 0x0020;
static const channel_t kChanConfidence = 0x0040;
static const channel_t kChanFlow = 0x0080;		// CV_16SC2 (format 10.5) from NVOF
static const channel_t kChanEnergy = 0x0100;

// should l/r gray be removed (not that expensive to re-calculate if needed)?
static const channel_t kChanLeftGray = 0x0200;	// CV_8UC1
static const channel_t kChanRightGray = 0x0400;	// CV_8UC1

static const channel_t kChanOverlay1 = 0x1000;

// maximum number of available channels
static const unsigned int n_channels = 13;

inline bool isFloatChannel(ftl::rgbd::channel_t chan) {
	return (chan == ftl::rgbd::kChanDepth || chan == ftl::rgbd::kChanEnergy);
}

// TODO:	interpolation for scaling depends on channel type;
//			NN for depth/disparity/optflow, linear/cubic/etc. for RGB

class Frame;

class Frame {
public:
	Frame() :	channels_host_(n_channels),
				channels_gpu_(n_channels),
				available_(n_channels, 0)
	{}

	/* @brief	Reset all channels without releasing memory.
	 */
	void reset()
	{
		std::fill(available_.begin(), available_.end(), 0);
	}

	/* @brief	Is there valid data in channel (either host or gpu).
	 */
	bool hasChannel(const ftl::rgbd::channel_t& channel)
	{
		return (channel == ftl::rgbd::kChanNone) ? true : available_[_channelIdx(channel)];
	}

	/* @brief	Method to get reference to the channel content
	 * @param	Channel type
	 * @param	CUDA stream
	 * @returns	Const reference to channel data
	 * 
	 * Result is valid only if hasChannel() is true. Host/Gpu transfer is
	 * performed, if necessary, but only once unless channel contents is
	 * changed by calling setChannel(). Return value valid only if
	 * hasChannel(channel) is true.
	 */
	template <typename T> const T& getChannel(const ftl::rgbd::channel_t& channel, cv::cuda::Stream& stream);
	template <typename T> const T& getChannel(const ftl::rgbd::channel_t& channel);

	/* @brief	Method to set/modify channel content
	 * @param	Channel type
	 * @returns	Reference to channel data
	 * 
	 * Returns non-const reference to channel memory. Invalidates other copies
	 * of the data (host/gpu) for the specified channel, next time getChannel()
	 * is called a memory transfer may occur.
	 * 
	 * NOTE:	If user of setChannel<T>() wants to modify contents instead of
	 * 			replacing them, getChannel<T>() needs to be called first to
	 * 			ensure there is valid contents in the returned reference!
	 * 			(TODO: interface could be improved)
	 */
	template <typename T> T& setChannel(const ftl::rgbd::channel_t& channel);

private:

	static size_t _channelIdx(const ftl::rgbd::channel_t& channel)
	{
		switch(channel)
		{
			case kChanNone:				return 0;
			case kChanLeft:				return 1;
			case kChanDepth:			return 2;
			case kChanRight:			return 3;
			case kChanDisparity:		return 4;
			case kChanDeviation:		return 5;
			case kChanNormals:			return 6;
			case kChanConfidence:		return 7;
			case kChanFlow:				return 8;
			case kChanEnergy:			return 9;
			case kChanLeftGray:			return 11;
			case kChanRightGray:		return 12;
			// should not happen (error); returned index is kChanNone
			default:					return 0;
		}
	}

	std::vector<cv::Mat> channels_host_;
	std::vector<cv::cuda::GpuMat> channels_gpu_;

	// bitmasks for each channel stored in available_
	static const uint mask_host = 1;
	static const uint mask_gpu = 2;

	std::vector<uint> available_;
};

template<> const cv::Mat& Frame::getChannel(const ftl::rgbd::channel_t& channel, cv::cuda::Stream& stream);
template<> const cv::cuda::GpuMat& Frame::getChannel(const ftl::rgbd::channel_t& channel, cv::cuda::Stream& stream);

template<> const cv::Mat& Frame::getChannel(const ftl::rgbd::channel_t& channel);
template<> const cv::cuda::GpuMat& Frame::getChannel(const ftl::rgbd::channel_t& channel);

template<> cv::Mat& Frame::setChannel(const ftl::rgbd::channel_t& channel);
template<> cv::cuda::GpuMat& Frame::setChannel(const ftl::rgbd::channel_t& channel);

}
}

#endif // _FTL_RGBD_FRAME_HPP_