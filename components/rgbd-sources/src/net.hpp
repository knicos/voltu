#pragma once
#ifndef _FTL_RGBD_NET_HPP_
#define _FTL_RGBD_NET_HPP_

#include <ftl/config.h>

#include <ftl/net/universe.hpp>
#include <ftl/rgbd/source.hpp>
#include <ftl/rgbd/detail/abr.hpp>
#include <ftl/threads.hpp>
#include <ftl/rgbd/detail/netframe.hpp>
#include <ftl/codecs/decoder.hpp>
#include <string>

#ifdef HAVE_NVPIPE
#include <NvPipe.h>
#endif

namespace ftl {
namespace rgbd {
namespace detail {

static const int kDefaultFrameCount = 30;
static const int kLatencyThreshold = 5;		// Milliseconds delay considered as late
static const int kSlowFramesThreshold = 5;	// Number of late frames before change
static const int kAdaptationRate = 5000;	// Milliseconds between bitrate changes

/**
 * A two channel network streamed source for RGB-Depth.
 */
class NetSource : public detail::Source {
	public:
	explicit NetSource(ftl::rgbd::Source *);
	~NetSource();

	bool capture(int64_t ts) { return true; }
	bool retrieve() { return true; }
	bool compute(int n, int b);
	bool isReady();

	void setPose(const Eigen::Matrix4d &pose);
	Camera parameters(channel_t chan);

	void reset();

	private:
	bool has_calibration_;
	ftl::UUID peer_;
	std::atomic<int> N_;
	bool active_;
	std::string uri_;
	ftl::net::callback_t h_;
	SHARED_MUTEX mutex_;
	cv::Mat idepth_;
	float gamma_;
	int temperature_;
	int minB_;
	int maxN_;
	int default_quality_;
	ftl::rgbd::channel_t prev_chan_;

	ftl::rgbd::detail::ABRController abr_;
	int last_bitrate_;

	//#ifdef HAVE_NVPIPE
	//NvPipe *nv_channel1_decoder_;
	//NvPipe *nv_channel2_decoder_;
	//#endif

	ftl::codecs::Decoder *decoder_c1_;
	ftl::codecs::Decoder *decoder_c2_;

	// Adaptive bitrate functionality
	ftl::rgbd::detail::bitrate_t adaptive_;	 // Current adapted bitrate
	//unsigned int slow_log_;		// Bit count of delayed frames
	//int64_t last_br_change_;	// Time of last adaptive change

	NetFrameQueue queue_;

	bool _getCalibration(ftl::net::Universe &net, const ftl::UUID &peer, const std::string &src, ftl::rgbd::Camera &p, ftl::rgbd::channel_t chan);
	void _recv(const std::vector<unsigned char> &jpg, const std::vector<unsigned char> &d);
	void _recvPacket(short ttimeoff, const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &);
	//void _recvChunk(int64_t frame, short ttimeoff, uint8_t bitrate, int chunk, const std::vector<unsigned char> &jpg, const std::vector<unsigned char> &d);
	//void _recvVideo(int64_t ts, short ttimeoff, uint8_t bitrate, const std::vector<unsigned char> &chan1, const std::vector<unsigned char> &chan2);
	void _updateURI();
	//void _checkAdaptive(int64_t);
	void _createDecoder(int chan, const ftl::codecs::Packet &);
};

}
}
}

#endif  // _FTL_RGBD_NET_HPP_
