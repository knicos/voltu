#pragma once
#ifndef _FTL_RGBD_STEREOVIDEO_HPP_
#define _FTL_RGBD_STEREOVIDEO_HPP_

#include <ftl/rgbd/source.hpp>
#include <string>

namespace ftl {

namespace rgbd {
namespace detail {

class LocalSource;
class Calibrate;
class Disparity;

/**
 * RGBD source from either a stereo video file with left + right images, or
 * direct from two camera devices. A variety of algorithms are included for
 * calculating disparity, before converting to depth.  Calibration of the images
 * is also performed.
 */
class StereoVideoSource : public detail::Source {
	public:
	explicit StereoVideoSource(ftl::rgbd::Source*);
	StereoVideoSource(ftl::rgbd::Source*, const std::string &);
	~StereoVideoSource();

	bool grab();
	bool isReady();

	//const cv::Mat &getRight() const { return right_; }

	private:
	LocalSource *lsrc_;
	Calibrate *calib_;
	Disparity *disp_;
	bool ready_;
	cv::Mat left_;
	cv::Mat right_;
	cv::Mat mask_l_;
};

}
}
}

#endif  // _FTL_RGBD_STEREOVIDEO_HPP_
