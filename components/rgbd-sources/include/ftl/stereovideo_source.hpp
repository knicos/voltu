#pragma once
#ifndef _FTL_RGBD_STEREOVIDEO_HPP_
#define _FTL_RGBD_STEREOVIDEO_HPP_

#include <ftl/rgbd_source.hpp>
#include <string>

namespace ftl {

class LocalSource;
class Calibrate;
class Disparity;

namespace rgbd {

/**
 * RGBD source from either a stereo video file with left + right images, or
 * direct from two camera devices. A variety of algorithms are included for
 * calculating disparity, before converting to depth.  Calibration of the images
 * is also performed.
 */
class StereoVideoSource : public RGBDSource {
	public:
	StereoVideoSource(nlohmann::json &config, ftl::net::Universe *net);
	StereoVideoSource(nlohmann::json &config, const std::string &file);
	~StereoVideoSource();

	void grab();
	bool isReady();

	const cv::Mat &getRight() const { return right_; }

	static inline RGBDSource *create(nlohmann::json &config, ftl::net::Universe *net) {
		return new StereoVideoSource(config, net);
	}

	private:
	ftl::LocalSource *lsrc_;
	ftl::Calibrate *calib_;
	ftl::Disparity *disp_;
	bool ready_;
	cv::Mat left_;
	cv::Mat right_;
	cv::Mat mask_l_;
};

}
}

#endif  // _FTL_RGBD_STEREOVIDEO_HPP_
