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

Calibrate::Calibrate(nlohmann::json &config, cv::Size image_size, cv::cuda::Stream &stream) : ftl::Configurable(config) {
	if (!_loadCalibration()) {
		throw ftl::exception("Loading calibration failed");
	}
	_calculateRectificationParameters(image_size);

	LOG(INFO) << "Calibration loaded from file";
	rectify_ = value("rectify", true);;

	this->on("use_intrinsics", [this](const ftl::config::Event &e) {
		rectify_ = value("rectify", true);
	});
}

bool Calibrate::_loadCalibration() {
	FileStorage fs;

	// read intrinsic parameters
	auto ifile = ftl::locateFile(value("intrinsics", std::string("intrinsics.yml")));
	if (ifile) {
		fs.open((*ifile).c_str(), FileStorage::READ);
		if (!fs.isOpened()) {
			LOG(WARNING) << "Could not open intrinsics file";
			return false;
		}

		LOG(INFO) << "Intrinsics from: " << *ifile;
	}
	else {
		LOG(WARNING) << "Calibration intrinsics file not found";
		return false;
	}

	fs["K"] >> K_;
	fs["D"] >> D_;
	fs["resolution"] >> calib_size_;

	if ((K_.size() != 2) || (D_.size() != 2)) {
		LOG(ERROR) << "Not enough intrinsic paramters, expected 2";
		return false;
	}

	fs.release();

	if (calib_size_.empty()) {
		LOG(ERROR) << "Calibration resolution missing";
		return false;
	}

	for (const Mat &K : K_) {
		if (K.size() != Size(3, 3)) {
			LOG(ERROR) << "Invalid intrinsic parameters";
			return false;
		}
	}
	for (const Mat &D : D_) {
		if (D.size() != Size(5, 1)) {
			LOG(ERROR) << "Invalid intrinsic parameters";
			return false;
		}
	}

	// read extrinsic parameters
	auto efile = ftl::locateFile(value("extrinsics", std::string("extrinsics.yml")));
	if (efile) {
		fs.open((*efile).c_str(), FileStorage::READ);
		if (!fs.isOpened()) {
			LOG(ERROR) << "Could not open extrinsics file";
			return false;
		}

		LOG(INFO) << "Extrinsics from: " << *efile;
	}
	else {
		LOG(ERROR) << "Calibration extrinsics file not found";
		return false;
	}

	fs["R"] >> R_;
	fs["T"] >> T_;
	fs["pose"] >> pose_;

	if (pose_.size() != Size(4, 4)) {
		LOG(ERROR) << "Pose not in calibration (using identity)";
		// TODO: return false (raises exception in constructor)
		//		 use config option to make pose optional (and not return false)

		pose_ = Mat::eye(Size(4, 4), CV_64FC1);
	}

	if ((R_.size() != Size(3, 3)) ||
		(T_.size() != Size(1, 3))) {

		LOG(ERROR) << "Invalid extrinsic parameters";
		return false;
	}
	fs.release();

	return true;
}

void Calibrate::_calculateRectificationParameters(Size img_size) {
	
	img_size_ = img_size;
	Mat K1 = _getK(0, img_size);
	Mat D1 = D_[0];
	Mat K2 = _getK(1, img_size);
	Mat D2 = D_[1];
	double alpha = value("alpha", 0.0);

	cv::stereoRectify(	K1, D1, K2, D2,
						img_size, R_, T_,
						R1_, R2_, P1_, P2_, Q_, 0, alpha);
	
	// TODO use fixed point maps for CPU (gpu remap() requires floating point)
	initUndistortRectifyMap(K1, D1, R1_, P1_, img_size, CV_32FC1, map1_.first, map2_.first);
	initUndistortRectifyMap(K2, D2, R2_, P2_, img_size, CV_32FC1, map1_.second, map2_.second);

	// CHECK Is this thread safe!!!!
	map1_gpu_.first.upload(map1_.first);
	map1_gpu_.second.upload(map1_.second);
	map2_gpu_.first.upload(map2_.first);
	map2_gpu_.second.upload(map2_.second);
}

void Calibrate::updateCalibration(const ftl::rgbd::Camera &p) {
	Q_.at<double>(3, 2) = 1.0 / p.baseline;
	Q_.at<double>(2, 3) = p.fx;
	Q_.at<double>(0, 3) = p.cx;
	Q_.at<double>(1, 3) = p.cy;

	// FIXME:(Nick) Update camera matrix also...
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

static Mat scaleCameraIntrinsics(Mat K, Size size_new, Size size_old) {
	Mat S(cv::Size(3, 3), CV_64F, 0.0);
	double scale_x = ((double) size_new.width) / ((double) size_old.width);
	double scale_y = ((double) size_new.height) / ((double) size_old.height);

	S.at<double>(0, 0) = scale_x;
	S.at<double>(1, 1) = scale_y;
	S.at<double>(2, 2) = 1.0;
	return S * K;
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
	Mat M;
	if (rectify_) {
		M = Mat(P1_, cv::Rect(0, 0, 3, 3));
	} else {
		M = K_[0];
	}
	return scaleCameraIntrinsics(M, res, img_size_);
}

cv::Mat Calibrate::getCameraMatrixRight(const cv::Size res) {
	Mat M;
	if (rectify_) {
		M = Mat(P2_, cv::Rect(0, 0, 3, 3));
	} else {
		M = K_[1];
	}
	return scaleCameraIntrinsics(M, res, img_size_);
}
