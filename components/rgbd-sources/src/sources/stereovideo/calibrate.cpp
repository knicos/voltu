/*
 * Copyright 2019 Nicolas Pope
 */

#include <loguru.hpp>
#include <ftl/config.h>
#include <ftl/configuration.hpp>
#include <ftl/threads.hpp>

#include <iostream>
#include <sstream>
#include <string>
#include <ctime>

#include "calibrate.hpp"
#include "ftl/exception.hpp"

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

using ftl::rgbd::detail::Calibrate;

using cv::FileStorage;

using cv::INTER_LINEAR;

using cv::FileNode;
using cv::FileNodeIterator;

using cv::Mat;
using cv::cuda::GpuMat;
using cv::cuda::Stream;

using cv::Size;

using cv::Point2f;
using cv::Point3f;
using cv::Matx33d;
using cv::Scalar;

using std::string;
using std::vector;

////////////////////////////////////////////////////////////////////////////////

static bool isValidTranslationForRectification(const Mat t) {
	if (t.type() != CV_64F)				{ return false; }
	if (t.channels() != 1) 				{ return false; }
	if (t.size() != Size(1, 3))			{ return false; }
	if (cv::norm(t, cv::NORM_L2) == 0)	{ return false; }
	return true;
}

static bool isValidRotationMatrix(const Mat M) {
	if (M.type() != CV_64F)				{ return false; }
	if (M.channels() != 1) 				{ return false; }
	if (M.size() != Size(3, 3))			{ return false; }

	if (abs(cv::determinant(M) - 1.0) > 0.00001)
										{ return false; } 

	// accuracy problems 
	// rotation matrix is orthogonal: M.T * M == M * M.T == I
	//if (cv::countNonZero((M.t() * M) != Mat::eye(Size(3, 3), CV_64FC1)) != 0)
	//									{ return false; }
	
	return true;
}

static bool isValidPose(const Mat M) {
	if (M.size() != Size(4, 4))			{ return false; }
	// check last row: 0 0 0 1
	return isValidRotationMatrix(M(cv::Rect(0 , 0, 3, 3)));
}

static bool isValidCamera(const Mat M) {
	if (M.type() != CV_64F)				{ return false; }
	if (M.channels() != 1)				{ return false; }
	if (M.size() != Size(3, 3))			{ return false; }
	// TODO: last row should be (0 0 0 1) ...
	return true;
}

static Mat scaleCameraIntrinsics(Mat K, Size size_new, Size size_old) {
	Mat S(cv::Size(3, 3), CV_64F, 0.0);
	double scale_x = ((double) size_new.width) / ((double) size_old.width);
	double scale_y = ((double) size_new.height) / ((double) size_old.height);

	S.at<double>(0, 0) = scale_x;
	S.at<double>(1, 1) = scale_y;
	S.at<double>(2, 2) = 1.0;
	return S * K;
}

////////////////////////////////////////////////////////////////////////////////

Calibrate::Calibrate(nlohmann::json &config, Size image_size, cv::cuda::Stream &stream) :
		ftl::Configurable(config) {
	
	img_size_ = image_size;
	calib_size_ = image_size;

	K_ = vector<Mat>(2);
	K_[0] = Mat::eye(Size(3, 3), CV_64FC1);
	K_[1] = Mat::eye(Size(3, 3), CV_64FC1);
	D_ = vector<Mat>(2);
	D_[0] = Mat::zeros(Size(5, 1), CV_64FC1);
	D_[1] = Mat::zeros(Size(5, 1), CV_64FC1);
	pose_ = Mat::eye(Size(4, 4), CV_64FC1);
	Q_ = Mat::eye(Size(4, 4), CV_64FC1);
	Q_.at<double>(3, 2) = -1;
	Q_.at<double>(2, 3) = 1;

	setRectify(true);
}

Mat Calibrate::_getK(size_t idx, Size size) {
	CHECK(idx < K_.size());
	CHECK(!size.empty());
	return scaleCameraIntrinsics(K_[idx], size, calib_size_);
}

Mat Calibrate::_getK(size_t idx) {
	return _getK(idx, img_size_);
}

cv::Mat Calibrate::getCameraMatrixLeft(const cv::Size res) {
	if (rectify_) {
		return Mat(P1_, cv::Rect(0, 0, 3, 3));
	} else {
		return scaleCameraIntrinsics(K_[0], res, img_size_);
	}
}

cv::Mat Calibrate::getCameraMatrixRight(const cv::Size res) {
	if (rectify_) {
		return Mat(P2_, cv::Rect(0, 0, 3, 3));
	} else {
		return scaleCameraIntrinsics(K_[1], res, img_size_);
	}
}

bool Calibrate::setRectify(bool enabled) {
	if (t_.empty() || R_.empty()) { enabled = false; }
	if (enabled) { 
		rectify_ = calculateRectificationParameters(); 
	}
	else {
		rectify_ = false;
	}
	return rectify_;
}

bool Calibrate::setIntrinsics(const Size size, const vector<Mat> K, const vector<Mat> D) {
	if (size.empty() || size.width <= 0 || size.height <= 0) { return false; }
	if ((K.size() != 2) || (D.size() != 2)) { return false; }
	for (const auto k : K) { if (!isValidCamera(k)) { return false; }}
	for (const auto d : D) { if (d.size() != Size(5, 1)) { return false; }}

	calib_size_ = size;
	K[0].copyTo(K_[0]);
	K[1].copyTo(K_[1]);
	D[0].copyTo(D_[0]);
	D[1].copyTo(D_[1]);
	return true;
}

bool Calibrate::setExtrinsics(const Mat R, const Mat t) {
	if (!isValidRotationMatrix(R) ||
		!isValidTranslationForRectification(t)) { return false; }
	
	R.copyTo(R_);
	t.copyTo(t_);
	return true;
}

bool Calibrate::setPose(const Mat P) {
	if (!isValidPose(P)) { return false; }
	P.copyTo(pose_);
	return true;
}

bool Calibrate::loadCalibration(const string fname) {
	FileStorage fs;

	fs.open((fname).c_str(), FileStorage::READ);
	if (!fs.isOpened()) {
		LOG(WARNING) << "Could not open calibration file";
		return false;
	}

	Size calib_size;
	vector<Mat> K;
	vector<Mat> D;
	Mat R;
	Mat t;
	Mat pose;

	fs["resolution"] >> calib_size;
	fs["K"] >> K;
	fs["D"] >> D;
	fs["R"] >> R;
	fs["t"] >> t;
	fs["P"] >> pose;
	fs.release();

	if (calib_size.empty()) {
		LOG(ERROR) << "calibration resolution missing in calibration file";
		return false;
	}
	if (!setIntrinsics(calib_size, K, D)) {
		LOG(ERROR) << "invalid intrinsics in calibration file";
		return false;
	}
	if (!setExtrinsics(R, t)) {
		LOG(ERROR) << "invalid extrinsics in calibration file";
		return false;
	}
	if (!setPose(pose)) {
		LOG(ERROR) << "invalid pose in calibration file";
		return false; // TODO: allow missing pose? (config option)
	}

	LOG(INFO) << "calibration loaded from: " << fname;
	return true;
}

bool Calibrate::writeCalibration(	const string fname, const Size size,
									const vector<Mat> K, const vector<Mat> D, 
									const Mat R, const Mat t, const Mat pose) {
	
	cv::FileStorage fs(fname, cv::FileStorage::WRITE);
	if (!fs.isOpened()) { return false; }

	fs	<< "resolution" << size
		<< "K" << K
		<< "D" << D
		<< "R" << R
		<< "t" << t
		<< "P" << pose
	;
	
	fs.release();
	return true;
}

bool Calibrate::saveCalibration(const string fname) {
	// note: never write rectified parameters!
	return writeCalibration(fname, calib_size_, K_, D_, R_, t_, pose_);
}

bool Calibrate::calculateRectificationParameters() {
	
	Mat K1 = _getK(0, img_size_);
	Mat D1 = D_[0];
	Mat K2 = _getK(1, img_size_);
	Mat D2 = D_[1];
	double alpha = value("alpha", 0.0);

	try {
		cv::stereoRectify(	K1, D1, K2, D2,
							img_size_, R_, t_,
							R1_, R2_, P1_, P2_, Q_, 0, alpha);
		
		// TODO use fixed point maps for CPU (gpu remap() requires floating point)
		initUndistortRectifyMap(K1, D1, R1_, P1_, img_size_, CV_32FC1, map1_.first, map2_.first);
		initUndistortRectifyMap(K2, D2, R2_, P2_, img_size_, CV_32FC1, map1_.second, map2_.second);
	}
	catch (cv::Exception ex) {
		LOG(ERROR) << ex.what();
		return false;
	}

	// CHECK Is this thread safe!!!!
	map1_gpu_.first.upload(map1_.first);
	map1_gpu_.second.upload(map1_.second);
	map2_gpu_.first.upload(map2_.first);
	map2_gpu_.second.upload(map2_.second);

	return true;
}

void Calibrate::rectifyStereo(GpuMat &l, GpuMat &r, Stream &stream) {
	if (!rectify_) { return; }
	// cv::cuda::remap() can not use same Mat for input and output
	// TODO: create tmp buffers only once
	GpuMat l_tmp(l.size(), l.type());
	GpuMat r_tmp(r.size(), r.type());
	cv::cuda::remap(l, l_tmp, map1_gpu_.first, map2_gpu_.first, cv::INTER_LINEAR, 0, cv::Scalar(), stream);
	cv::cuda::remap(r, r_tmp, map1_gpu_.second, map2_gpu_.second, cv::INTER_LINEAR, 0, cv::Scalar(), stream);
	stream.waitForCompletion();
	l = l_tmp;
	r = r_tmp;
}

void Calibrate::rectifyStereo(cv::Mat &l, cv::Mat &r) {
	if (!rectify_) { return; }
	// cv::cuda::remap() can not use same Mat for input and output
	cv::remap(l, l, map1_.first, map2_.first, cv::INTER_LINEAR, 0, cv::Scalar());
	cv::remap(r, r, map1_.second, map2_.second, cv::INTER_LINEAR, 0, cv::Scalar());
}
