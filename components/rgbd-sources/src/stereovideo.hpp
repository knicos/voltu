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

	void swap();
	bool capture(int64_t ts);
	bool retrieve();
	bool compute(int n, int b);
	bool isReady();
	Camera parameters(channel_t chan);

	//const cv::Mat &getRight() const { return right_; }

	private:
	LocalSource *lsrc_;
	Calibrate *calib_;
	Disparity *disp_;
	
	bool ready_;
	bool use_optflow_;
	
	cv::cuda::Stream stream_;
	cv::cuda::Stream stream2_;

	std::vector<Frame> frames_;

	cv::Mat mask_l_;

#ifdef HAVE_OPTFLOW
	// see comments in https://gitlab.utu.fi/nicolas.pope/ftl/issues/155
	cv::Ptr<cv::cuda::NvidiaOpticalFlow_1_0> nvof_;
	cv::cuda::GpuMat optflow_;
#endif

	void init(const std::string &);
};

}
}
}

#endif  // _FTL_RGBD_STEREOVIDEO_HPP_
