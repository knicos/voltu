#pragma once
#ifndef _FTL_CALIBRATION_DATA_HPP_
#define _FTL_CALIBRATION_DATA_HPP_

#include <vector>
#include <opencv2/core/core.hpp>

#define _NDISTORTION_PARAMETERS 3
#define _NCAMERA_PARAMETERS (9 + _NDISTORTION_PARAMETERS)

namespace ftl {
namespace calibration {

/**
 * Camera paramters
 */
struct Camera {
	Camera() {}
	Camera(const cv::Mat& K, const cv::Mat& D, const cv::Mat& R, const cv::Mat& tvec);

	void setRotation(const cv::Mat& R);
	void setTranslation(const cv::Mat& tvec);
	void setExtrinsic(const cv::Mat& R, const cv::Mat& t) {
		setRotation(R);
		setTranslation(t);
	}

	void setIntrinsic(const cv::Mat& K);
	void setDistortion(const cv::Mat &D);
	void setIntrinsic(const cv::Mat& K, const cv::Mat& D) {
		setIntrinsic(K);
		setDistortion(D);
	}

	cv::Mat intrinsicMatrix() const;
	cv::Mat distortionCoefficients() const;

	cv::Mat rvec() const;
	cv::Mat tvec() const;
	cv::Mat rmat() const;

	cv::Mat extrinsicMatrix() const;
	cv::Mat extrinsicMatrixInverse() const;

	double data[_NCAMERA_PARAMETERS] = {0.0};

	enum Parameter {
		ROTATION = 0,
		RX = 0,
		RY = 1,
		RZ = 2,
		TRANSLATION = 3,
		TX = 3,
		TY = 4,
		TZ = 5,
		F = 6,
		CX = 7,
		CY = 8,
		DISTORTION = 9,
		K1 = 9,
		K2 = 10,
		K3 = 11
	};
};

class CalibrationData {
public:
	CalibrationData() {};
	explicit CalibrationData(int n_cameras);

	void init(int n_cameras);

	int addObservation(const std::vector<bool>& visible, const std::vector<cv::Point2d>& observations);
	int addObservation(const std::vector<bool>& visible, const std::vector<cv::Point2d>& observations, const cv::Point3d& point);

	int addObservations(const std::vector<bool>& visible, const std::vector<std::vector<cv::Point2d>>& observations, const std::vector<cv::Point3d>& point);

	void reserve(int n_points) {
		points_.reserve(n_points);
		points_camera_.reserve(n_points*n_cameras_);
		observations_.reserve(n_points*n_cameras_);
		visible_.reserve(n_points*n_cameras_);
	}

	// TODO: method for save poinst_camera_, could return (for example)
	// vector<pair<Point3d*>, vector<Point3d*>>
	
	std::vector<std::vector<cv::Point2d>> getObservations(const std::vector<int> &cameras);
	/* Get points corresponding to observations returned by getObservations() or getObservationsPtr() */
	std::vector<std::reference_wrapper<cv::Point3d>> getPoints(const std::vector<int> &cameras);

	/* get reference/pointer to data */
	inline Camera& getCamera(int k) { return cameras_[k]; }
	inline std::vector<Camera>& getCameras() { return cameras_; }
	inline cv::Point3d& getPoint(int i) { return points_[i]; }
	inline cv::Point2d& getObservation(int k, int i) { return observations_[n_cameras_*i+k]; }
	inline bool isVisible(int k, int i) { return visible_[n_cameras_*i+k]; }
	inline bool isVisible(const std::vector<int> &cameras, int i) {
		for (const auto &k : cameras) { if (!isVisible(k, i)) { return false; } }
		return true;
	}

	int npoints() const { return points_.size(); }
	int ncameras() const { return n_cameras_; }

private:
	std::pair<std::vector<std::vector<cv::Point2d>>, std::vector<std::reference_wrapper<cv::Point3d>>> _getVisible(const std::vector<int> &cameras);
	
	int n_cameras_;

	// cameras
	std::vector<Camera> cameras_;
	// points for each observation
	std::vector<cv::Point3d> points_;
	// points estimated from cameras' observations
	std::vector<cv::Point3d> points_camera_;
	// observations
	std::vector<cv::Point2d> observations_;
	// visibility
	std::vector<bool> visible_;
};

}
}

#endif
