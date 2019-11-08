#pragma once
#ifndef _FTL_RGBD_STEREOVIDEO_HPP_
#define _FTL_RGBD_STEREOVIDEO_HPP_

#include <ftl/rgbd/source.hpp>
#include <ftl/operators/operator.hpp>
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
	Camera parameters(ftl::codecs::Channel chan);

	private:
	LocalSource *lsrc_;
	Calibrate *calib_;

	ftl::operators::Graph *pipeline_input_;
	ftl::operators::Graph *pipeline_depth_;

	bool ready_;
	
	cv::cuda::Stream stream_;
	cv::cuda::Stream stream2_;

	std::vector<Frame> frames_;

	cv::Mat mask_l_;

	void init(const std::string &);
};

}
}
}

#endif  // _FTL_RGBD_STEREOVIDEO_HPP_
