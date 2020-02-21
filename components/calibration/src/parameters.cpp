#include "ftl/calibration/parameters.hpp"

#include <opencv2/calib3d/calib3d.hpp>

using cv::Mat;
using cv::Size;
using cv::Point2d;
using cv::Point3d;
using cv::Vec3d;
using cv::Rect;

using std::vector;

using namespace ftl::calibration;

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
	if      (Camera::n_distortion_parameters <= 4)  { D = Mat::zeros(4, 1, CV_64FC1); }
	else if (Camera::n_distortion_parameters <= 5)  { D = Mat::zeros(5, 1, CV_64FC1); }
	else if (Camera::n_distortion_parameters <= 8)  { D = Mat::zeros(8, 1, CV_64FC1); }
	else if (Camera::n_distortion_parameters <= 12) { D = Mat::zeros(12, 1, CV_64FC1); }
	else if (Camera::n_distortion_parameters <= 14) { D = Mat::zeros(14, 1, CV_64FC1); }

	for (int i = 0; i < Camera::n_distortion_parameters; i++) {
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

bool validate::translationStereo(const Mat &t) {
	if (t.type() != CV_64F)				{ return false; }
	if (t.channels() != 1) 				{ return false; }
	if (t.size() != Size(1, 3))			{ return false; }
	if (cv::norm(t, cv::NORM_L2) == 0)	{ return false; }
	return true;
}

bool validate::rotationMatrix(const Mat &M) {
	if (M.type() != CV_64F)				{ return false; }
	if (M.channels() != 1) 				{ return false; }
	if (M.size() != Size(3, 3))			{ return false; }

	double det = cv::determinant(M);
	if (abs(abs(det)-1.0) > 0.00001)	{ return false; }

	// TODO: floating point errors (result not exactly identity matrix)
	// rotation matrix is orthogonal: M.T * M == M * M.T == I
	//if (cv::countNonZero((M.t() * M) != Mat::eye(Size(3, 3), CV_64FC1)) != 0)
	//									{ return false; }
	
	return true;
}

bool validate::pose(const Mat &M) {
	if (M.size() != Size(4, 4))			{ return false; }
	if (!validate::rotationMatrix(M(cv::Rect(0 , 0, 3, 3))))
										{ return false; }
	if (!(	(M.at<double>(3, 0) == 0.0) && 
			(M.at<double>(3, 1) == 0.0) && 
			(M.at<double>(3, 2) == 0.0) && 
			(M.at<double>(3, 3) == 1.0))) { return false; }

	return true;
}

bool validate::cameraMatrix(const Mat &M) {
	if (M.type() != CV_64F)				{ return false; }
	if (M.channels() != 1)				{ return false; }
	if (M.size() != Size(3, 3))			{ return false; }
	
	if (!(	(M.at<double>(2, 0) == 0.0) && 
			(M.at<double>(2, 1) == 0.0) && 
			(M.at<double>(2, 2) == 1.0))) { return false; }
	
	return true;
}

bool ftl::calibration::validate::distortionCoefficients(const Mat &D, Size size) {
	if (D.type() != CV_64FC1) { return false; }
	if (!(
		(D.total() == 4) ||
		(D.total() == 5) ||
		(D.total() == 8) ||
		(D.total() == 12))) { return false; }

	for (int i = 0; i < D.total(); i++) {
		if (!std::isfinite(D.at<double>(i))) { return false; }
	}

	double k[6] = {0.0};
	//double p[2] = {0.0};
	//double s[4] = {0.0};

	switch(D.total()) {
		case 12:
			/*
			s[0] = D.at<double>(8);
			s[1] = D.at<double>(9);
			s[2] = D.at<double>(10);
			s[3] = D.at<double>(11);
			*/
			[[fallthrough]];
		
		case 8:
			k[3] = D.at<double>(5);
			k[4] = D.at<double>(6);
			k[5] = D.at<double>(7);
			[[fallthrough]];

		case 5:
			k[2] = D.at<double>(4);
			[[fallthrough]];

		default:
			k[0] = D.at<double>(0);
			k[1] = D.at<double>(1);
			/*
			p[0] = D.at<double>(2);
			p[1] = D.at<double>(3);
			*/
	}
	
	int diagonal = sqrt(size.width*size.width+size.height*size.height) + 1.0;

	bool is_n = true;
	bool is_p = true;

	double dist_prev_n = 0;
	double dist_prev_p = 0;

	for (int r = 0; r < diagonal; r++) {
		double r2 = r*r;
		double r4 = r2*r2;
		double r6 = r4*r2;

		double rdist = 1.0 + k[0]*r2 + k[1]*r4 + k[2]*r6;
		double irdist2 = 1./(1.0 + k[3]*r2 + k[4]*r4 + k[5]*r6);
		double dist = r2*rdist*irdist2; // + s[0]*r2 + s[1]*r4;

		if (is_n) {
			if (r2 == 0) {}
			else if (!(dist < dist_prev_n)) { is_n = false; }
			dist_prev_n = dist;
		}

		if (is_p) {
			if (r2 == 0) {}
			else if (!(dist > dist_prev_p)) { is_p = false; }
			dist_prev_p = dist;
		}

		if (!is_n && !is_p) { return false; }
	}
	
	return true;
}

Mat ftl::calibration::scaleCameraMatrix(const Mat &K, const Size &size_new, const Size &size_old) {
	Mat S(cv::Size(3, 3), CV_64F, 0.0);
	double scale_x = ((double) size_new.width) / ((double) size_old.width);
	double scale_y = ((double) size_new.height) / ((double) size_old.height);

	S.at<double>(0, 0) = scale_x;
	S.at<double>(1, 1) = scale_y;
	S.at<double>(2, 2) = 1.0;
	return (S*K);
}

double ftl::calibration::reprojectionError(const vector<Point2d>& points_im,
	const vector<Point3d>& points, const Mat& K, const Mat& D, const Mat& R,
	const Mat& t) {

	Mat rvec;
	if (R.size() == Size(3, 3)) { cv::Rodrigues(R, rvec); }
	else { rvec = R; }

	vector<Point2d> points_reprojected;
	cv::projectPoints(points, rvec, t, K, D, points_reprojected);

	double err = 0.0;
	int npoints = points_im.size();

	for (int i = 0; i < npoints; i++) {
		err += norm(points_im[i] - points_reprojected[i]);
	}

	return err / ((double) npoints);
}
