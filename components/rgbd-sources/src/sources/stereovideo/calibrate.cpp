/*
 * Copyright 2019 Nicolas Pope
 */

#include <loguru.hpp>
#include <ftl/config.h>
#include <ftl/configuration.hpp>
#include <ftl/threads.hpp>
#include <ftl/calibration/parameters.hpp>

#include "calibrate.hpp"
#include "ftl/exception.hpp"

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/cudawarping.hpp>

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
	pose_adjustment_ = Mat::eye(Size(4, 4), CV_64FC1);
	Q_ = Mat::eye(Size(4, 4), CV_64FC1);
	Q_.at<double>(3, 2) = -1;
	Q_.at<double>(2, 3) = 1;

	setRectify(true);
}

Mat Calibrate::_getK(size_t idx, Size size) {
	CHECK(idx < K_.size());
	CHECK(!size.empty());
	return ftl::calibration::scaleCameraMatrix(K_[idx], size, calib_size_);
}

Mat Calibrate::_getK(size_t idx) {
	return _getK(idx, img_size_);
}

double Calibrate::getBaseline() const {
	if (t_.empty()) { return 0.0; }
	return cv::norm(t_);
}

double Calibrate::getDoff() const {
	return -(Q_.at<double>(3,3) * getBaseline());
}

double Calibrate::getDoff(const Size& size) const {
	return getDoff() * ((double) size.width / (double) img_size_.width);
}

Mat Calibrate::getCameraMatrixLeft(const cv::Size res) {
	if (rectify_) {
		return ftl::calibration::scaleCameraMatrix(Mat(P1_, cv::Rect(0, 0, 3, 3)), res, img_size_);
	} else {
		return ftl::calibration::scaleCameraMatrix(K_[0], res, calib_size_);
	}
}

Mat Calibrate::getCameraMatrixRight(const cv::Size res) {
	if (rectify_) {
		return ftl::calibration::scaleCameraMatrix(Mat(P2_, cv::Rect(0, 0, 3, 3)), res, img_size_);
	} else {
		return ftl::calibration::scaleCameraMatrix(K_[1], res, calib_size_);
	}
}

Mat Calibrate::getCameraDistortionLeft() {
	if (rectify_) {	return Mat::zeros(Size(5, 1), CV_64FC1); }
	else { return D_[0]; }
}

Mat Calibrate::getCameraDistortionRight() {
	if (rectify_) {	return Mat::zeros(Size(5, 1), CV_64FC1); }
	else { return D_[1]; }
}

Mat Calibrate::getPose() const {
	Mat T;
	if (rectify_) {
		Mat R1 = Mat::eye(4, 4, CV_64FC1);
		R1_.copyTo(R1(cv::Rect(0, 0, 3, 3)));
		T = pose_ * R1.inv();
	}
	else {
		pose_.copyTo(T);
	}
	return pose_adjustment_ * T;
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

bool Calibrate::setDistortion(const vector<Mat> &D) {
	if (D.size() != 2) { return false; }
	for (const auto d : D) { if (d.size() != Size(5, 1)) { return false; }}
	D[0].copyTo(D_[0]);
	D[1].copyTo(D_[1]);
	return true;
}

bool Calibrate::setIntrinsics(const Size &size, const vector<Mat> &K) {
	if (K.size() != 2) { return false; }
	if (size.empty() || size.width <= 0 || size.height <= 0) { return false; }
	for (const auto k : K) {
		if (!ftl::calibration::validate::cameraMatrix(k)) {
			return false;
		}
	}

	calib_size_ = Size(size);
	K[0].copyTo(K_[0]);
	K[1].copyTo(K_[1]);
	return true;
}

bool Calibrate::setExtrinsics(const Mat &R, const Mat &t) {
	if (!ftl::calibration::validate::rotationMatrix(R) ||
		!ftl::calibration::validate::translationStereo(t)) { return false; }
	
	R.copyTo(R_);
	t.copyTo(t_);
	return true;
}

bool Calibrate::setPose(const Mat &P) {
	if (!ftl::calibration::validate::pose(P)) { return false; }
	P.copyTo(pose_);
	return true;
}

bool Calibrate::setPoseAdjustment(const Mat &T) {
	if (!ftl::calibration::validate::pose(T)) { return false; }
	pose_adjustment_ = T * pose_adjustment_;
	return true;
}

bool Calibrate::loadCalibration(const string &fname) {
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
	Mat pose_adjustment;

	fs["resolution"] >> calib_size;
	fs["K"] >> K;
	fs["D"] >> D;
	fs["R"] >> R;
	fs["t"] >> t;
	fs["P"] >> pose;
	fs["adjustment"] >> pose_adjustment;
	fs.release();

	bool retval = true;
	if (calib_size.empty()) {
		LOG(ERROR) << "calibration resolution missing in calibration file";
		retval = false;
	}
	if (!setIntrinsics(calib_size, K)) {
		LOG(ERROR) << "invalid intrinsics in calibration file";
		retval = false;
	}
	if (!setDistortion(D)) {
		LOG(ERROR) << "invalid distortion parameters in calibration file";
		retval = false;
	}
	if (!setExtrinsics(R, t)) {
		LOG(ERROR) << "invalid extrinsics in calibration file";
		retval = false;
	}
	if (!setPose(pose)) {
		LOG(ERROR) << "invalid pose in calibration file";
		retval = false;
	}
	if (!setPoseAdjustment(pose_adjustment)) {
		LOG(WARNING) << "invalid pose adjustment in calibration file (using identity)";
	}

	LOG(INFO) << "calibration loaded from: " << fname;
	return retval;
}

bool Calibrate::writeCalibration(	const string &fname, const Size &size,
									const vector<Mat> &K, const vector<Mat> &D, 
									const Mat &R, const Mat &t, const Mat &pose,
									const Mat &pose_adjustment) {
	
	cv::FileStorage fs(fname, cv::FileStorage::WRITE);
	if (!fs.isOpened()) { return false; }

	fs	<< "resolution" << size
		<< "K" << K
		<< "D" << D
		<< "R" << R
		<< "t" << t
		<< "P" << pose
		<< "adjustment" << pose_adjustment;
	;
	
	fs.release();
	return true;
}

bool Calibrate::saveCalibration(const string &fname) {
	// note: never write rectified parameters!

	// TODO: make a backup of old file
	//if (std::filesystem::is_regular_file(fname)) {
	//	// copy to fname + ".bak"
	//}

	return writeCalibration(fname, calib_size_, K_, D_, R_, t_, pose_, pose_adjustment_);
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
		
		initUndistortRectifyMap(K1, D1, R1_, P1_, img_size_, CV_32FC1, map1_.first, map2_.first);
		initUndistortRectifyMap(K2, D2, R2_, P2_, img_size_, CV_32FC1, map1_.second, map2_.second);
		
		// CHECK Is this thread safe!!!!
		map1_gpu_.first.upload(map1_.first);
		map1_gpu_.second.upload(map1_.second);
		map2_gpu_.first.upload(map2_.first);
		map2_gpu_.second.upload(map2_.second);

		Mat map0 = map1_.first.clone();
		Mat map1 = map2_.first.clone();
		cv::convertMaps(map0, map1, map1_.first, map2_.first, CV_16SC2);

		map0 = map1_.second.clone();
		map1 = map2_.second.clone();
		cv::convertMaps(map0, map1, map1_.second, map2_.second, CV_16SC2);
	}
	catch (cv::Exception &ex) {
		LOG(ERROR) << ex.what();
		return false;
	}

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

void Calibrate::rectifyLeft(cv::Mat &l) {
	if (!rectify_) { return; }
	cv::remap(l, l, map1_.first, map2_.first, cv::INTER_LINEAR, 0, cv::Scalar());
}

void Calibrate::rectifyRight(cv::Mat &r) {
	if (!rectify_) { return; }
	cv::remap(r, r, map1_.second, map2_.second, cv::INTER_LINEAR, 0, cv::Scalar());
}
