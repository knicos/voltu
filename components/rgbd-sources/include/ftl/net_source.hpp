#pragma once
#ifndef _FTL_RGBD_NET_HPP_
#define _FTL_RGBD_NET_HPP_

#include <ftl/net/universe.hpp>
#include <ftl/rgbd_source.hpp>
#include <string>

namespace ftl {
namespace rgbd {

/**
 * RGBD source from either a stereo video file with left + right images, or
 * direct from two camera devices. A variety of algorithms are included for
 * calculating disparity, before converting to depth.  Calibration of the images
 * is also performed.
 */
class NetSource : public RGBDSource {
	public:
	explicit NetSource(nlohmann::json &config);
	NetSource(nlohmann::json &config, ftl::net::Universe *net);
	~NetSource();

	void grab();
	bool isReady();

	static inline RGBDSource *create(nlohmann::json &config, ftl::net::Universe *net) {
		return new NetSource(config, net);
	}

	private:
	bool has_calibration_;
};

}
}

#endif  // _FTL_RGBD_NET_HPP_
