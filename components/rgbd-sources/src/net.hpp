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

	bool grab(int n, int b);
	bool isReady();

	void setPose(const Eigen::Matrix4d &pose);

	void reset();

	private:
	bool has_calibration_;
	ftl::UUID peer_;
	int N_;
	bool active_;
	std::string uri_;
	ftl::net::callback_t h_;
	MUTEX mutex_;
	int chunks_dim_;
	int chunk_width_;
	int chunk_height_;
	cv::Mat idepth_;
	float gamma_;
	int temperature_;
	int minB_;
	int maxN_;
	int default_quality_;
	ftl::rgbd::channel_t prev_chan_;

	bool _getCalibration(ftl::net::Universe &net, const ftl::UUID &peer, const std::string &src, ftl::rgbd::Camera &p);
	void _recv(const std::vector<unsigned char> &jpg, const std::vector<unsigned char> &d);
	void _recvChunk(int frame, int chunk, bool delta, const std::vector<unsigned char> &jpg, const std::vector<unsigned char> &d);
	void _updateURI();
};

}
}
}

#endif  // _FTL_RGBD_NET_HPP_
