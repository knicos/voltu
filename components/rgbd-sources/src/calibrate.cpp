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
	std::pair<Mat, Mat> map1, map2;
	calibrated_ = _loadCalibration(image_size, map1, map2);

	if (calibrated_) {
		_updateIntrinsics();
		LOG(INFO) << "Calibration loaded from file";
	}
	else {
		LOG(WARNING) << "Calibration not loaded";
	}

	this->on("use_intrinsics", [this](const ftl::config::Event &e) {
		_updateIntrinsics();
	});
}

bool Calibrate::_loadCalibration(cv::Size img_size, std::pair<Mat, Mat> &map1, std::pair<Mat, Mat> &map2) {
	FileStorage fs;

	// reading intrinsic parameters
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

	{
		vector<Mat> K, D;
		fs["K"] >> K;
		fs["D"] >> D;

		K[0].copyTo(M1_);
		K[1].copyTo(M2_);
		D[0].copyTo(D1_);
		D[1].copyTo(D2_);
	}

	fs.release();

	CHECK(M1_.size() == Size(3, 3));
	CHECK(M2_.size() == Size(3, 3));
	CHECK(D1_.size() == Size(5, 1));
	CHECK(D2_.size() == Size(5, 1));

	auto efile = ftl::locateFile(value("extrinsics", std::string("extrinsics.yml")));
	if (efile) {
		fs.open((*efile).c_str(), FileStorage::READ);
		if (!fs.isOpened()) {
			LOG(WARNING) << "Could not open extrinsics file";
			return false;
		}

		LOG(INFO) << "Extrinsics from: " << *efile;
	}
	else {
		LOG(WARNING) << "Calibration extrinsics file not found";
		return false;
	}

	fs["R"] >> R_;
	fs["T"] >> T_;
	fs["R1"] >> R1_;
	fs["R2"] >> R2_;
	fs["P1"] >> P1_;
	fs["P2"] >> P2_;
	fs["Q"] >> Q_;

	fs.release();

	img_size_ = img_size;

	// TODO: normalize calibration
	double scale_x = ((double) img_size.width) / 1280.0;
	double scale_y = ((double) img_size.height) / 720.0;
	
	Mat scale(cv::Size(3, 3), CV_64F, 0.0);
	scale.at<double>(0, 0) = scale_x;
	scale.at<double>(1, 1) = scale_y;
	scale.at<double>(2, 2) = 1.0;

	M1_ = scale * M1_;
	M2_ = scale * M2_;
	P1_ = scale * P1_;
	P2_ = scale * P2_;

	Q_.at<double>(0, 3) = Q_.at<double>(0, 3) * scale_x;
	Q_.at<double>(1, 3) = Q_.at<double>(1, 3) * scale_y;
	Q_.at<double>(2, 3) = Q_.at<double>(2, 3) * scale_x; // TODO: scaling?
	Q_.at<double>(3, 3) = Q_.at<double>(3, 3) * scale_x;

	// cv::cuda::remap() works only with CV_32FC1
	initUndistortRectifyMap(M1_, D1_, R1_, P1_, img_size_, CV_32FC1, map1.first, map2.first);
	initUndistortRectifyMap(M2_, D2_, R2_, P2_, img_size_, CV_32FC1, map1.second, map2.second);

	return true;
}

void Calibrate::updateCalibration(const ftl::rgbd::Camera &p) {
	std::pair<Mat, Mat> map1, map2;

	Q_.at<double>(3, 2) = 1.0 / p.baseline;
	Q_.at<double>(2, 3) = p.fx;
	Q_.at<double>(0, 3) = p.cx;
	Q_.at<double>(1, 3) = p.cy;

	// FIXME:(Nick) Update camera matrix also...
	_updateIntrinsics();
}

void Calibrate::_updateIntrinsics() {
	// TODO: pass parameters?

	Mat R1, R2, P1, P2;
	ftl::rgbd::Camera params();

	if (this->value("use_intrinsics", true)) {
		// rectify
		R1 = R1_;
		R2 = R2_;
		P1 = P1_;
		P2 = P2_;
	}
	else {
		// no rectification
		R1 = Mat::eye(Size(3, 3), CV_64FC1);
		R2 = R1;
		P1 = M1_;
		P2 = M2_;
	}

	// Set correct camera matrices for
	// getCameraMatrix(), getCameraMatrixLeft(), getCameraMatrixRight()
	C_l_ = P1;
	C_r_ = P2;

	initUndistortRectifyMap(M1_, D1_, R1, P1, img_size_, CV_32FC1, map1_.first, map2_.first);
	initUndistortRectifyMap(M2_, D2_, R2, P2, img_size_, CV_32FC1, map1_.second, map2_.second);

	// CHECK Is this thread safe!!!!
	map1_gpu_.first.upload(map1_.first);
	map1_gpu_.second.upload(map1_.second);
	map2_gpu_.first.upload(map2_.first);
	map2_gpu_.second.upload(map2_.second);
}

void Calibrate::rectifyStereo(GpuMat &l, GpuMat &r, Stream &stream) {
	// cv::cuda::remap() can not use same Mat for input and output

	GpuMat l_tmp(l.size(), l.type());
	GpuMat r_tmp(r.size(), r.type());
	cv::cuda::remap(l, l_tmp, map1_gpu_.first, map2_gpu_.first, cv::INTER_LINEAR, 0, cv::Scalar(), stream);
	cv::cuda::remap(r, r_tmp, map1_gpu_.second, map2_gpu_.second, cv::INTER_LINEAR, 0, cv::Scalar(), stream);
	stream.waitForCompletion();
	l = l_tmp;
	r = r_tmp;
}

void Calibrate::rectifyStereo(cv::Mat &l, cv::Mat &r) {
	// cv::cuda::remap() can not use same Mat for input and output

	cv::remap(l, l, map1_.first, map2_.first, cv::INTER_LINEAR, 0, cv::Scalar());
	cv::remap(r, r, map1_.second, map2_.second, cv::INTER_LINEAR, 0, cv::Scalar());

	/*GpuMat l_tmp(l.size(), l.type());
	GpuMat r_tmp(r.size(), r.type());
	cv::cuda::remap(l, l_tmp, map1_gpu_.first, map2_gpu_.first, cv::INTER_LINEAR, 0, cv::Scalar(), stream);
	cv::cuda::remap(r, r_tmp, map1_gpu_.second, map2_gpu_.second, cv::INTER_LINEAR, 0, cv::Scalar(), stream);
	stream.waitForCompletion();
	l = l_tmp;
	r = r_tmp;*/
}

bool Calibrate::isCalibrated() {
	return calibrated_;
}