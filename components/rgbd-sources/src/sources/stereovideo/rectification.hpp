/*
 * Copyright 2019 Nicolas Pope
 */

#ifndef _FTL_CALIBRATION_HPP_
#define _FTL_CALIBRATION_HPP_

#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>

#include <string>
#include <vector>

#include <ftl/codecs/channels.hpp>
#include <ftl/rgbd/camera.hpp>
#include <ftl/calibration/structures.hpp>

namespace ftl {
namespace rgbd {
namespace detail {

/**
 * Stereo rectification. Performs rectification for left and right channels.
 * Rectified image is same size as input image.
 */
class StereoRectification : public ftl::Configurable {
public:
	StereoRectification(nlohmann::json &config, cv::Size image_size);

	void setInterpolation(int interpolation);

	void setSize(cv::Size);
	/**
	 * Calculate rectification parameters from given calibration.
	 */
	void setCalibration(ftl::calibration::CalibrationData &calib);
	bool calibrated();

	void rectify(cv::InputOutputArray im, ftl::codecs::Channel c);

	/**
	 * Enable/disable rectification.
	 */
	void setEnabled(bool enabled);
	bool enabled();

	/**
	 * Get camera pose (rectified if enabled and valid)
	 */
	cv::Mat getPose(ftl::codecs::Channel c = ftl::codecs::Channel::Left);

	/**
	 * Get intrinsic matrix.
	 */
	cv::Mat cameraMatrix(ftl::codecs::Channel c = ftl::codecs::Channel::Left);
	cv::Mat cameraMatrix(cv::Size size, ftl::codecs::Channel c = ftl::codecs::Channel::Left);

	/** Stereo baseline */
	double baseline();
	/** Disparity offset */
	double doff();
	double doff(cv::Size);

protected:
	void calculateParameters();

private:
	cv::Size calib_resolution_;
	ftl::calibration::CalibrationData::Calibration calib_left_;
	ftl::calibration::CalibrationData::Calibration calib_right_;

	cv::Size image_resolution_;

	// rectification parameters and maps
	bool enabled_;
	bool valid_;
	int interpolation_;
	double baseline_;
	cv::Mat Q_;
	cv::Mat R_l_;
	cv::Mat R_r_;
	cv::Mat P_l_;
	cv::Mat P_r_;

	std::pair<cv::Mat,cv::Mat> map_l_;
	std::pair<cv::Mat,cv::Mat> map_r_;

	// temporary buffers
	// cv::cuda::HostMem tmp_l_;
	// cv::cuda::HostMem tmp_r_;
	cv::Mat tmp_l_;
	cv::Mat tmp_r_;
};

}
}
}

#endif // _FTL_CALIBRATION_HPP_

