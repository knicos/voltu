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

	bool grab(int n, int b);
	bool isReady();
	Camera parameters(channel_t chan);

	//const cv::Mat &getRight() const { return right_; }

	private:
	LocalSource *lsrc_;
	Calibrate *calib_;
	Disparity *disp_;
	
	bool ready_;
	
	cv::cuda::Stream stream_;

	cv::cuda::GpuMat left_;
	cv::cuda::GpuMat right_;
	cv::cuda::GpuMat disp_tmp_;
	cv::cuda::GpuMat depth_tmp_;
	
	cv::Mat mask_l_;

	void init(const std::string &);
};

}
}
}

#endif  // _FTL_RGBD_STEREOVIDEO_HPP_
