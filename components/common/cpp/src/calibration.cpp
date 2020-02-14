#include "ftl/calibration.hpp"

using cv::Mat;
using cv::Size;

using namespace ftl::calibration;

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