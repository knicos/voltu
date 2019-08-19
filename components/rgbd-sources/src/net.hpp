#pragma once
#ifndef _FTL_RGBD_NET_HPP_
#define _FTL_RGBD_NET_HPP_

#include <ftl/net/universe.hpp>
#include <ftl/rgbd/source.hpp>
#include <ftl/threads.hpp>
#include <string>

namespace ftl {
namespace rgbd {
namespace detail {

static const int kDefaultFrameCount = 30;

/**
 * Buffers for a single frame as it is being received over the network.
 */
struct NetFrame {
	cv::Mat channel1;
	cv::Mat channel2;
	volatile int64_t timestamp;
	std::atomic<int> chunk_count;
	MUTEX mtx;
};

/**
 * Manage multiple frames with their timestamp as an identifier. Once a frame
 * is completed it should be freed from the queue for reuse.
 */
class NetFrameQueue {
	public:
	explicit NetFrameQueue(int size=2);
	~NetFrameQueue();

	NetFrame &getFrame(int64_t ts, const cv::Size &, int c1type, int c2type);
	void freeFrame(NetFrame &);

	private:
	std::vector<NetFrame> frames_;
	MUTEX mtx_;
};

/**
 * RGBD source from either a stereo video file with left + right images, or
 * direct from two camera devices. A variety of algorithms are included for
 * calculating disparity, before converting to depth.  Calibration of the images
 * is also performed.
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
	int chunks_dim_;
	int chunk_width_;
	int chunk_height_;
	cv::Mat idepth_;
	float gamma_;
	int temperature_;
	int minB_;
	int maxN_;
	int default_quality_;
	int chunk_count_;
	ftl::rgbd::channel_t prev_chan_;
	//volatile int64_t current_frame_;
	//std::atomic<int> chunk_count_;

	// Double buffering
	//cv::Mat d_depth_;
	//cv::Mat d_rgb_;

	NetFrameQueue queue_;

	bool _getCalibration(ftl::net::Universe &net, const ftl::UUID &peer, const std::string &src, ftl::rgbd::Camera &p, ftl::rgbd::channel_t chan);
	void _recv(const std::vector<unsigned char> &jpg, const std::vector<unsigned char> &d);
	void _recvChunk(int64_t frame, int chunk, bool delta, const std::vector<unsigned char> &jpg, const std::vector<unsigned char> &d);
	void _updateURI();
};

}
}
}

#endif  // _FTL_RGBD_NET_HPP_
