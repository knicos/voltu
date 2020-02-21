#pragma once
#ifndef _FTL_CALIBRATION_DATA_HPP_
#define _FTL_CALIBRATION_DATA_HPP_

#include <vector>
#include <opencv2/core/core.hpp>
#include <ftl/calibration/optimize.hpp>

#define _NDISTORTION_PARAMETERS 3
#define _NCAMERA_PARAMETERS (9 + _NDISTORTION_PARAMETERS)

namespace ftl {
namespace calibration {

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
