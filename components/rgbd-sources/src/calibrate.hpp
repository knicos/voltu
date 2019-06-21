/*
 * Copyright 2019 Nicolas Pope
 */

#ifndef _FTL_CALIBRATION_HPP_
#define _FTL_CALIBRATION_HPP_

#include <opencv2/opencv.hpp>
#include "local.hpp"
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace cv {
class FileStorage;
class FileNode;
};

namespace ftl {
namespace rgbd {
namespace detail {

/**
 * Manage local calibration details: undistortion, rectification and camera parameters.
 * Calibration parameters can be acquired using ftl-calibrate app.
 */
class Calibrate : public ftl::Configurable {
	public:
	Calibrate(nlohmann::json &config, cv::Size image_size, cv::cuda::Stream &stream);
	
	/**
	 * Get both left and right images from local source, but with intrinsic
	 * and extrinsic stereo calibration already applied.
	 */
	bool rectified(cv::cuda::GpuMat &l, cv::cuda::GpuMat &r, cv::cuda::Stream &stream);

	/**
	 * Rectify and remove distortions from from images l and r using cv::remap()
	 */
	void rectifyStereo(cv::cuda::GpuMat &l, cv::cuda::GpuMat &r, cv::cuda::Stream &stream);

	bool isCalibrated(); // TODO ???

	/**
	 * Get the camera matrix. Used to convert disparity map back to depth and
	 * a 3D point cloud.
	 */
	const cv::Mat &getQ() const { return Q_; }
	const cv::Mat &getCameraMatrix() const { return P_; }

private:
	bool _loadCalibration(cv::Size img_size, std::pair<cv::Mat, cv::Mat> &map1, std::pair<cv::Mat, cv::Mat> &map2);
	
	private:
	bool calibrated_;

	std::pair<cv::cuda::GpuMat, cv::cuda::GpuMat> map1_gpu_;
	std::pair<cv::cuda::GpuMat, cv::cuda::GpuMat> map2_gpu_;

	cv::Mat P_;
	cv::Mat Q_;
};

}
}
}

#endif // _FTL_CALIBRATION_HPP_

