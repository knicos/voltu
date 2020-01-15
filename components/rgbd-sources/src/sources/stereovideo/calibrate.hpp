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
#include <ftl/rgbd/camera.hpp>

namespace cv {
class FileStorage;
class FileNode;
};

namespace ftl {
namespace rgbd {
namespace detail {

/**
 * Manage local calibration details: undistortion, rectification and camera
 * parameters.
 */
class Calibrate : public ftl::Configurable {
	public:
	Calibrate(nlohmann::json &config, cv::Size image_size, cv::cuda::Stream &stream);

	/* @brief	Rectify and undistort stereo pair images (GPU)
	 */
	void rectifyStereo(cv::cuda::GpuMat &l, cv::cuda::GpuMat &r, cv::cuda::Stream &stream);

	/* @brief	Rectify and undistort stereo pair images (CPU)
	 * @todo	Uses same rectification maps as GPU version, according to OpenCV
	 * 			documentation for remap(), fixed point versions faster for CPU
	 */
	void rectifyStereo(cv::Mat &l, cv::Mat &r);

	void updateCalibration(const ftl::rgbd::Camera &p);
	
	/* @brief Get disparity to depth matrix
	 *
	 * 2020/01/15:	Not used, StereoVideoSource creates a Camera object which
	 * 				is used to calculate depth from disparity (disp2depth.cu)
	 */
	const cv::Mat &getQ() const { return Q_; }

	/* @brief	Get intrinsic paramters for rectified camera
	 * @param	Camera resolution
	 */
	cv::Mat getCameraMatrixLeft(const cv::Size res);
	cv::Mat getCameraMatrixRight(const cv::Size res);

	/* @brief	Get camera pose from calibration
	 */
	cv::Mat getPose() { return pose_; };
	
	/* @brief	Enable/disable recitification. If disabled, instance returns
	 *			original camera intrinsic parameters (getCameraMatrixLeft() and
				getCameraMatrixRight() methods). When enabled (default), those
				methods return camera parameters for rectified images.
	 * @param	Rectification on/off
	 */
	void setRectify(bool enabled) { rectify_ = enabled; }

private:
	// rectification enabled/disabled
	bool rectify_;

	/* @brief	Get intrinsic matrix saved in calibration.
	 * @param	Camera index (0 left, 1 right)
	 * @param	Resolution
	 */
	cv::Mat _getK(size_t idx, cv::Size size);
	cv::Mat _getK(size_t idx);

	/* @brief	Calculate rectification parameters and maps
	 * @param	Camera resolution
	 */
	void _calculateRectificationParameters(cv::Size img_size);

	/* @brief	Load calibration from file
	 * @todo	File names as arguments
	 */
	bool _loadCalibration();
	
	// calibration resolution (loaded from file by _loadCalibration)
	cv::Size calib_size_;
	// camera resolution (set by _calculateRecitificationParameters)
	cv::Size img_size_;

	// rectification maps
	std::pair<cv::Mat, cv::Mat> map1_;
	std::pair<cv::Mat, cv::Mat> map2_;
	std::pair<cv::cuda::GpuMat, cv::cuda::GpuMat> map1_gpu_;
	std::pair<cv::cuda::GpuMat, cv::cuda::GpuMat> map2_gpu_;

	// transformation from left to right camera: R_ and T_
	cv::Mat R_;
	cv::Mat T_;
	// pose for left camera
	cv::Mat pose_;

	// parameters for rectification, see cv::stereoRectify() documentation
	cv::Mat R1_;
	cv::Mat P1_;
	cv::Mat R2_;
	cv::Mat P2_;

	// disparity to depth matrix
	cv::Mat Q_;
	
	// intrinsic parameters and distortion coefficients
	std::vector<cv::Mat> K_;
	std::vector<cv::Mat> D_;
};

}
}
}

#endif // _FTL_CALIBRATION_HPP_

