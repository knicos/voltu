#pragma once
#ifndef _FTL_RGBD_STEREOVIDEO_HPP_
#define _FTL_RGBD_STEREOVIDEO_HPP_

#include "../../basesource.hpp"
#include <ftl/operators/operator.hpp>
#include <string>

namespace ftl {

namespace rgbd {
namespace detail {

class Device;
class Calibrate;
class Disparity;

/**
 * RGBD source from either a stereo video file with left + right images, or
 * direct from two camera devices. 
 */
class StereoVideoSource : public BaseSourceImpl {
	public:
	explicit StereoVideoSource(ftl::rgbd::Source*);
	StereoVideoSource(ftl::rgbd::Source*, const std::string &);
	~StereoVideoSource();

	bool capture(int64_t ts) override;
	bool retrieve(ftl::rgbd::Frame &frame) override;
	bool isReady() override;

	Camera parameters(ftl::codecs::Channel chan) override;

	private:
	void updateParameters();

	Device *lsrc_;
	Calibrate *calib_;
	int64_t capts_;

	cv::Size color_size_;
	cv::Size depth_size_;

	ftl::operators::Graph *pipeline_input_;
	ftl::operators::Graph *pipeline_depth_;

	cv::cuda::GpuMat fullres_left_;
	cv::cuda::GpuMat fullres_right_;

	bool ready_;
	
	cv::cuda::Stream stream_;
	cv::cuda::Stream stream2_;

	cv::Mat mask_l_;

	void init(const std::string &);
};

}
}
}

#endif  // _FTL_RGBD_STEREOVIDEO_HPP_
