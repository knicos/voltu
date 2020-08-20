#pragma once
#ifndef _FTL_RGBD_STEREOVIDEO_HPP_
#define _FTL_RGBD_STEREOVIDEO_HPP_

#include "../../basesource.hpp"
#include <ftl/operators/operator.hpp>
#include <ftl/calibration/structures.hpp>
#include <string>
#include <memory>

namespace ftl {

namespace rgbd {
namespace detail {

class Device;
class StereoRectification;
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

	static bool supported(const std::string &dev);

private:
	void updateParameters(ftl::rgbd::Frame &);

	Device *lsrc_;
	std::unique_ptr<StereoRectification> rectification_;
	ftl::calibration::CalibrationData calibration_;

	int64_t capts_;

	//cv::Size color_size_;
	cv::Size depth_size_;

	ftl::operators::Graph *pipeline_input_=nullptr;
	ftl::operators::Graph *pipeline_depth_;

	cv::cuda::GpuMat fullres_left_;
	cv::cuda::GpuMat fullres_right_;

	bool ready_;
	bool do_update_params_ = false;
	bool cap_status_ = false;

	cv::Mat mask_l_;
	cudaStream_t stream_;

	ftl::Handle calibration_change_;
	std::string fname_calib_;

	void init(const std::string &);
};

}
}
}

#endif  // _FTL_RGBD_STEREOVIDEO_HPP_
