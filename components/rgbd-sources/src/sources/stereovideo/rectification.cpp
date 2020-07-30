/*
 * Copyright 2019 Nicolas Pope
 */

#include <loguru.hpp>
#include <ftl/config.h>
#include <ftl/configuration.hpp>
#include <ftl/calibration/parameters.hpp>

#include "rectification.hpp"
#include "ftl/exception.hpp"

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

using ftl::rgbd::detail::StereoRectification;
using ftl::calibration::CalibrationData;
using ftl::codecs::Channel;

StereoRectification::StereoRectification(nlohmann::json &config, cv::Size image_size) :
	ftl::Configurable(config), image_resolution_(image_size),
	enabled_(false), valid_(false), interpolation_(cv::INTER_LINEAR),
	baseline_(0.0) {

	map_l_.first.create(image_resolution_, map_format_);
	map_l_.second.create(image_resolution_, map_format_);
	map_r_.first.create(image_resolution_, map_format_);
	map_r_.second.create(image_resolution_, map_format_);
}

void StereoRectification::setInterpolation(int interpolation) {
	interpolation_ = interpolation;
}

void StereoRectification::setEnabled(bool value) {
	enabled_ = value;
}

bool StereoRectification::enabled() {
	return enabled_;
}

bool StereoRectification::calibrated() {
	return valid_;
}

void StereoRectification::setCalibration(CalibrationData &calib) {
	if (calib.hasCalibration(Channel::Left) && calib.hasCalibration(Channel::Right)) {
		calib_left_ = calib.get(Channel::Left);
		calib_right_ = calib.get(Channel::Right);
		updateCalibration_();
	}
}

void StereoRectification::updateCalibration_() {
	using namespace ftl::calibration;
	// TODO: lock
	{
		bool valid = true;
		valid &= calib_left_.intrinsic.resolution != cv::Size{0, 0};
		valid &= calib_right_.intrinsic.resolution != cv::Size{0, 0};
		valid &= validate::cameraMatrix(calib_left_.intrinsic.matrix());
		valid &= validate::cameraMatrix(calib_right_.intrinsic.matrix());
		valid &= (calib_left_.extrinsic.tvec != calib_right_.extrinsic.tvec);
		if (!valid) { return; }
	}

	valid_ = false;

	// create temporary buffers for rectification
	if (tmp_l_.size() != image_resolution_) {
		tmp_l_ = cv::Mat(image_resolution_, CV_8UC4);
	}
	if (tmp_l_.size() != image_resolution_) {
		tmp_r_ = cv::Mat(image_resolution_, CV_8UC4);
	}

	// calculate rotation and translation from left to right using calibration
	cv::Mat T_l = calib_left_.extrinsic.matrix();
	cv::Mat T_r = calib_right_.extrinsic.matrix();
	cv::Mat T = T_r * transform::inverse(T_l);

	transform::getRotationAndTranslation(T, R_, t_);
	baseline_ = cv::norm(t_);

	if (baseline_ == 0.0) { return; }
	valid_ = true;
	calculateParameters_();
}

void StereoRectification::calculateParameters_() {
	if (!valid_) { return; }

	cv::Mat K_l = calib_left_.intrinsic.matrix(image_resolution_);
	cv::Mat K_r = calib_right_.intrinsic.matrix(image_resolution_);
	cv::Mat dc_l = calib_left_.intrinsic.distCoeffs.Mat();
	cv::Mat dc_r = calib_right_.intrinsic.distCoeffs.Mat();

	// calculate rectification parameters
	cv::stereoRectify(	K_l, dc_l, K_r, dc_r, image_resolution_,
						R_, t_, R_l_, R_r_, P_l_, P_r_, Q_, 0, 0);

	cv::initUndistortRectifyMap(K_l, dc_l, R_l_, P_l_, image_resolution_,
								map_format_, map_l_.first, map_l_.second);
	cv::initUndistortRectifyMap(K_r, dc_r, R_r_, P_r_, image_resolution_,
								map_format_, map_r_.first, map_r_.second);

}

void StereoRectification::rectify(cv::InputArray im, cv::OutputArray im_out, Channel c) {

	if (!enabled_ || !valid_) {
		im.copyTo(im_out);
		return;
	}

	if (im.size() != image_resolution_) {
		throw ftl::exception("Input has wrong size");
	}

	if (im.isMat()) {
		if (!im_out.isMat()) {
			throw ftl::exception(	"Input and Output arrays must have same "
									"type (cv::Mat expected)");
		}
		cv::Mat in = im.getMat();
		cv::Mat &out = im_out.getMatRef(); // assumes valid size/type

		if (c == Channel::Left) {
			cv::remap(in, out, map_l_.first, map_l_.second, interpolation_);
		}
		else if (c == Channel::Right) {
			cv::remap(in, out, map_r_.first, map_r_.second, interpolation_);
		}
		else {
			throw ftl::exception("Bad channel for rectification");
		}
	}
	else if (im.isGpuMat()) {
		throw ftl::exception("GPU rectification not implemented");
	}
	else {
		throw ftl::exception("Input not Mat/GpuMat");
	}
}

cv::Mat StereoRectification::getPose(Channel c) {
	// NOTE: FTL poses are camera-to-world transformations while the parameters
	//		 in calibration are world-to-camera. cv::stereoRectify() rotation
	//		 is unrectified-to-rectified.

	using ftl::calibration::transform::inverse;

	if (enabled_ && valid_) {
		cv::Mat R = cv::Mat::eye(4, 4, CV_64FC1);
		if (c == Channel::Left) {
			R_l_.copyTo(R(cv::Rect(0, 0, 3, 3)));
			return inverse(R * calib_left_.extrinsic.matrix());
		}
		else if (c == Channel::Right) {
			R_r_.copyTo(R(cv::Rect(0, 0, 3, 3)));
			return inverse(R * calib_right_.extrinsic.matrix());
		}
	}
	else {
		if (c == Channel::Left) {
			return inverse(calib_left_.extrinsic.matrix());
		}
		else if (c == Channel::Right) {
			return inverse(calib_right_.extrinsic.matrix());
		}
	}
	throw ftl::exception("Invalid channel, expected Left or Right");
}

double StereoRectification::baseline() {
	return baseline_;
}

double StereoRectification::doff() {
	if (!enabled_ || !valid_) return 0.0;
	return -(Q_.at<double>(3,3) * baseline_);
}

double StereoRectification::doff(cv::Size size) {
	return doff() * double(size.width)/double(image_resolution_.width);
}

cv::Mat StereoRectification::cameraMatrix(Channel c) {
	if (enabled_ && valid_) {
		if (c == Channel::Left) {
			// P_l_: Left camera is origin in rectified system, there extrinsic
			// is no rotation and intrinsic matrix can be directly extracted.
			return cv::Mat(P_l_, cv::Rect(0, 0, 3, 3)).clone();
		}
		else if (c == Channel::Right) {
			// Extrinsics are included in P_r_, can't do same as above
			throw ftl::exception("Not implemented");
		}
	}
	else {
		if (c == Channel::Left) {
			return calib_left_.intrinsic.matrix();
		}
		else if (c == Channel::Right) {
			return calib_right_.intrinsic.matrix();
		}
	}
	throw ftl::exception("Invalid channel, expected Left or Right");
}

cv::Mat StereoRectification::cameraMatrix(cv::Size size, Channel c) {
	return ftl::calibration::scaleCameraMatrix(cameraMatrix(c), image_resolution_, size);
}
