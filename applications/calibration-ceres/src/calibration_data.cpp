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
using ftl::calibration::Camera;

////////////////////////////////////////////////////////////////////////////////

void Camera::setRotation(const Mat& R) {
	if (((R.size() != Size(3, 3)) &&
		(R.size() != Size(3, 1)) &&
		(R.size() != Size(1, 3))) ||
		(R.type() != CV_64FC1)) { throw std::exception(); }

	Mat rvec;
	if (R.size() == cv::Size(3, 3)) { cv::Rodrigues(R, rvec); }
	else { rvec = R; }

	data[Parameter::RX] = rvec.at<double>(0);
	data[Parameter::RY] = rvec.at<double>(1);
	data[Parameter::RZ] = rvec.at<double>(2);
}

void Camera::setTranslation(const Mat& t) {
	if ((t.type() != CV_64FC1) ||
		(t.size() != cv::Size(1, 3))) { throw std::exception(); }

	data[Parameter::TX] = t.at<double>(0);
	data[Parameter::TY] = t.at<double>(1);
	data[Parameter::TZ] = t.at<double>(2);
}


void Camera::setIntrinsic(const Mat& K) {
	if ((K.type() != CV_64FC1) || (K.size() != cv::Size(3, 3))) {
		throw std::exception();
	}

	data[Parameter::F] = K.at<double>(0, 0);
	data[Parameter::CX] = K.at<double>(0, 2);
	data[Parameter::CY] = K.at<double>(1, 2);
}

void Camera::setDistortion(const Mat& D) {
	if ((D.type() != CV_64FC1)) { throw std::exception(); }
	switch(D.total()) {
		case 12:
			/*
			data[Parameter::S1] = D.at<double>(8);
			data[Parameter::S2] = D.at<double>(9);
			data[Parameter::S3] = D.at<double>(10);
			data[Parameter::S4] = D.at<double>(11);
			*/
			[[fallthrough]];
		
		case 8:
			/*
			data[Parameter::K4] = D.at<double>(5);
			data[Parameter::K5] = D.at<double>(6);
			data[Parameter::K6] = D.at<double>(7);
			*/
			[[fallthrough]];

		case 5:
			data[Parameter::K3] = D.at<double>(4);
			[[fallthrough]];

		default:
			data[Parameter::K1] = D.at<double>(0);
			data[Parameter::K2] = D.at<double>(1);
			/*
			data[Parameter::P1] = D.at<double>(2);
			data[Parameter::P2] = D.at<double>(3);
			*/
	}
}

Camera::Camera(const Mat &K, const Mat &D, const Mat &R, const Mat &tvec) {
	setIntrinsic(K, D);
	if (!R.empty()) { setRotation(R); }
	if (!tvec.empty()) { setTranslation(tvec); }
}

Mat Camera::intrinsicMatrix() const {
	Mat K = Mat::eye(3, 3, CV_64FC1);
	K.at<double>(0, 0) = data[Parameter::F];
	K.at<double>(1, 1) = data[Parameter::F];
	K.at<double>(0, 2) = data[Parameter::CX];
	K.at<double>(1, 2) = data[Parameter::CY];
	return K;
}

Mat Camera::distortionCoefficients() const {
	Mat D;
	// OpenCV distortion parameter sizes: 4, 5, 8, 12, 14
	if      (_NDISTORTION_PARAMETERS <= 4)  { D = Mat::zeros(4, 1, CV_64FC1); }
	else if (_NDISTORTION_PARAMETERS <= 5)  { D = Mat::zeros(5, 1, CV_64FC1); }
	else if (_NDISTORTION_PARAMETERS <= 8)  { D = Mat::zeros(8, 1, CV_64FC1); }
	else if (_NDISTORTION_PARAMETERS <= 12) { D = Mat::zeros(12, 1, CV_64FC1); }
	else if (_NDISTORTION_PARAMETERS <= 14) { D = Mat::zeros(14, 1, CV_64FC1); }

	for (int i = 0; i < _NDISTORTION_PARAMETERS; i++) {
		D.at<double>(i) = data[Parameter::DISTORTION+i];
	}
	return D;
}

Mat Camera::rvec() const {
	return Mat(Vec3d(data[Parameter::RX], data[Parameter::RY], data[Parameter::RZ]));
}

Mat Camera::tvec() const {
	return Mat(Vec3d(data[Parameter::TX], data[Parameter::TY], data[Parameter::TZ]));
}

Mat Camera::rmat() const {
	Mat R;
	cv::Rodrigues(rvec(), R);
	return R;
}

Mat Camera::extrinsicMatrix() const {
	Mat T = Mat::eye(4, 4, CV_64FC1);
	rmat().copyTo(T(Rect(0, 0, 3, 3)));
	tvec().copyTo(T(Rect(3, 0, 1, 3)));
	return T;
}

Mat Camera::extrinsicMatrixInverse() const {
	return extrinsicMatrix().inv();
}

////////////////////////////////////////////////////////////////////////////////

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
