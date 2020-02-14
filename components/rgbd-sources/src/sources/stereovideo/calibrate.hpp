/*
 * Copyright 2019 Nicolas Pope
 */

#ifndef _FTL_CALIBRATION_HPP_
#define _FTL_CALIBRATION_HPP_

#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include "local.hpp"
#include <string>
#include <vector>
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

	/**
	 * @brief	Rectify and undistort stereo pair images (GPU)
	 */
	void rectifyStereo(cv::cuda::GpuMat &l, cv::cuda::GpuMat &r, cv::cuda::Stream &stream);

	/**
	 * @brief	Rectify and undistort stereo pair images (CPU)
	 */
	void rectifyStereo(cv::Mat &l, cv::Mat &r);

	/**
	 * @brief	Rectify and undistort left image (CPU)
	 */
	void rectifyLeft(cv::Mat &l);

	/**
	 * @brief	Rectify and undistort right image (CPU)
	 */
	void rectifyRight(cv::Mat &r);

	void updateCalibration(const ftl::rgbd::Camera &p);
	
	/**
	 * @brief Get disparity to depth matrix
	 *
	 * 2020/01/15:	StereoVideoSource creates a Camera object which is used to
	 * 				calculate depth from disparity (disp2depth.cu). Seems to be
	 * 				used only in StereoVideoSource to get doff and baseline
	 * 				parameter values in updateParameters()
	 */
	const cv::Mat &getQ() const { return Q_; }

	/**
	 * @brief Get camera pair baseline
	 */
	double getBaseline() const;

	/**
	 * @brief	Get intrinsic paramters. If rectification is enabled, returns
	 *			rectified intrinsic parameters, otherwise returns values from
	 *			calibration. Parameters are scaled for given resolution.
	 * @param	res		camera resolution
	 */
	cv::Mat getCameraMatrixLeft(const cv::Size res);
	/** @brief	Same as getCameraMatrixLeft() for right camera */
	cv::Mat getCameraMatrixRight(const cv::Size res);

	/** @brief	Get camera distortion parameters. If rectification is enabled,
	 * 			returns zeros. Otherwise returns calibrated distortion 
	 * 			parameters values.
	 */
	cv::Mat getCameraDistortionLeft();
	/** @brief	Same as getCameraDistortionLeft() for right camera */
	cv::Mat getCameraDistortionRight();

	/**
	 * @brief	Get camera pose from calibration. Returns pose to rectified
	 * 			camera if rectification is enabled.
	 */
	cv::Mat getPose() const;
	
	/**
	 * @brief	Enable/disable recitification. If disabled, instance returns
	 *			original camera intrinsic parameters (getCameraMatrixLeft() and
	 *			getCameraMatrixRight() methods). When enabled (default), those
	 *			methods return camera parameters for rectified images. Does not
	 *			enable rectification, if valid parameters are missing.
	 * @param	Rectification on/off
	 * @returns	Status after call
	 */
	bool setRectify(bool enabled);

	/**
	 * @brief	Set intrinsic parameters for both cameras.
	 * 
	 * @param	size	calibration size
	 * @param	K		2 camera matricies (3x3)
	 * @returns	true if valid parameters
	 */
	bool setIntrinsics(const cv::Size &size, const std::vector<cv::Mat> &K);

	/**
	 * @brief	Set lens distortion parameters
	 * @param	D 		2 distortion parameters (5x1)
	 */
	bool setDistortion(const std::vector<cv::Mat> &D);

	/**
	 * @brief	Set extrinsic parameters.
	 * 
	 * @param	R	Rotation matrix (3x3) from left to right camera
	 * @param	t	Translation vector (1x3) from left to right camera
	 * @returns	true if valid parameters
	 */
	bool setExtrinsics(const cv::Mat &R, const cv::Mat &t);

	/**
	 * @brief	Set pose
	 * @param	pose	Pose for left camera
	 * @returns	true if valid pose
	 */
	bool setPose(const cv::Mat &P);

	/**
	 * @brief	Calculate rectification parameters and maps. Can fail if
	 * 			calibration parameters are invalid.
	 * @returns	true if successful
	 */
	bool calculateRectificationParameters();

	/**
	 * @brief	Load calibration from file
	 * @param	fname	File name
	 */
	bool loadCalibration(const std::string &fname);

	/**
	 * @brief	Write calibration parameters to file
	 * 
	 * Assumes two cameras and intrinsic calibration parameters have the same
	 * resolution.
	 * 
	 * @todo	Validate loaded values
	 * 
	 * @param	fname file name
	 * @param	size calibration resolution (intrinsic parameters)
	 * @param	K intrinsic matrices
	 * @param	D distortion coefficients
	 * @param	R rotation from first camera to second
	 * @param	t translation from first camera to second
	 * @param	pose first camera's pose 
	 */
	static bool writeCalibration(const std::string &fname,
								const cv::Size &size,
								const std::vector<cv::Mat> &K,
								const std::vector<cv::Mat> &D,
								const cv::Mat &R, const cv::Mat &t,
								const cv::Mat &pose);

	/*	@brief	Save current calibration to file
	 *	@param	File name
	 */
	bool saveCalibration(const std::string &fname);

private:
	// rectification enabled/disabled
	volatile bool rectify_;

	/**
	 * @brief	Get intrinsic matrix saved in calibration.
	 * @param	Camera index (0 left, 1 right)
	 * @param	Resolution
	 */
	cv::Mat _getK(size_t idx, cv::Size size);
	cv::Mat _getK(size_t idx);

	// calibration resolution (loaded from file by loadCalibration)
	cv::Size calib_size_;
	// camera resolution
	cv::Size img_size_;

	// rectification maps
	std::pair<cv::Mat, cv::Mat> map1_;
	std::pair<cv::Mat, cv::Mat> map2_;
	std::pair<cv::cuda::GpuMat, cv::cuda::GpuMat> map1_gpu_;
	std::pair<cv::cuda::GpuMat, cv::cuda::GpuMat> map2_gpu_;

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

	// transformation from left to right camera: R_ and T_
	cv::Mat R_;
	cv::Mat t_;
	// pose for left camera
	cv::Mat pose_;
};

}
}
}

#endif // _FTL_CALIBRATION_HPP_

