/*
 * Copyright 2019 Nicolas Pope
 */

#include <loguru.hpp>
#include <ftl/config.h>
#include <ftl/configuration.hpp>

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
		LOG(INFO) << "Calibration loaded from file";
		map1_gpu_.first.upload(map1.first, stream);
		map1_gpu_.second.upload(map1.second, stream);
		map2_gpu_.first.upload(map2.first, stream);
		map2_gpu_.second.upload(map2.second, stream);
	}
	else {
		LOG(WARNING) << "Calibration not loaded";
	}
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

	Mat M1, D1, M2, D2;
	fs["M"] >> M1;
	fs["D"] >> D1;

	M2 = M1;
	D2 = D1;

	auto efile = ftl::locateFile("extrinsics.yml");
	if (efile) {
		fs.open((*efile).c_str(), FileStorage::READ);
		if (!fs.isOpened()) {
			LOG(WARNING) << "Could not open extrinsics file";
			return false;
		}
		
		LOG(INFO) << "Extrinsics from: " << *efile;
	} else {
		LOG(WARNING) << "Calibration extrinsics file not found";
		return false;
	}

	Mat R, T, R1, P1, R2, P2;
	fs["R"] >> R;
	fs["T"] >> T;
	fs["R1"] >> R1;
	fs["R2"] >> R2;
	fs["P1"] >> P1;
	fs["P2"] >> P2;
	fs["Q"] >> Q_;

	P_ = P1;

	// cv::cuda::remap() works only with CV_32FC1
	initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_32FC1, map1.first, map2.first);
	initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_32FC1, map1.second, map2.second);

	return true;
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

bool Calibrate::isCalibrated() {
	return calibrated_;
}