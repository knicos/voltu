#include "calibration_data.hpp"

#include <opencv2/calib3d.hpp>

using std::vector;
using std::reference_wrapper;
using std::pair;
using std::make_pair;

using cv::Mat;
using cv::Point2d;
using cv::Point3d;
using cv::Vec3d;
using cv::Rect;
using cv::Size;

using ftl::calibration::CalibrationData;


CalibrationData::CalibrationData(int n_cameras) { init(n_cameras); }

void CalibrationData::init(int n_cameras) {
	n_cameras_ = n_cameras;
	cameras_ = vector<Camera>(n_cameras);
}

int CalibrationData::addObservation(const vector<bool> &visible, const vector<Point2d> &observations) {
	if ((int) observations.size() != n_cameras_) { throw std::exception(); }
	if ((int) visible.size() != n_cameras_) { throw std::exception(); }
	visible_.insert(visible_.end(), visible.begin(), visible.end());
	observations_.insert(observations_.end(), observations.begin(), observations.end());
	points_.push_back(Point3d(0.0, 0.0, 0.0));

	return npoints() - 1;
}

int CalibrationData::addObservation(const vector<bool> &visible, const vector<Point2d> &observations, const Point3d &point) {
	if ((int) observations.size() != n_cameras_) { throw std::exception(); }
	if ((int) visible.size() != n_cameras_) { throw std::exception(); }
	visible_.insert(visible_.end(), visible.begin(), visible.end());
	observations_.insert(observations_.end(), observations.begin(), observations.end());
	points_.push_back(point);

	return npoints() - 1;
}

int CalibrationData::addObservations(const vector<bool>& visible, const vector<vector<Point2d>>& observations, const vector<Point3d>& points) {
	int retval = -1;
	for (size_t i = 0; i < observations.size(); i++) {
		retval = addObservation(visible, observations[i], points[i]);
	}
	return retval;
}

pair<vector<vector<Point2d>>, vector<reference_wrapper<Point3d>>> CalibrationData::_getVisible(const vector<int> &cameras) {

	int n_points = npoints();
	vector<vector<Point2d>> observations(cameras.size());
	vector<reference_wrapper<Point3d>> points;

	for (size_t k = 0; k < cameras.size(); k++) {
		observations[k].reserve(n_points);
	}
	points.reserve(n_points);

	for (int i = 0; i < n_points; i++) {
		if (!isVisible(cameras, i)) { continue; }

		for (size_t k = 0; k < cameras.size(); k++) {
			observations[k].push_back(getObservation(cameras[k], i));
		}
		points.push_back(getPoint(i));
	}

	return make_pair(observations, points);
}

vector<vector<Point2d>> CalibrationData::getObservations(const vector<int> &cameras) {
	return _getVisible(cameras).first;
}

vector<reference_wrapper<Point3d>> CalibrationData::getPoints(const vector<int> &cameras) {
	return _getVisible(cameras).second;
}
