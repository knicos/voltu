/*
 * Copyright 2019 Nicolas Pope
 */

#include <glog/logging.h>

#include <string>
#include <chrono>

#include <ftl/local.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

using ftl::LocalSource;
using cv::Mat;
using cv::VideoCapture;
using cv::Rect;
using std::string;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::high_resolution_clock;

LocalSource::LocalSource(nlohmann::json &config)
		: timestamp_(0.0),
		flip_(config["flip"]),
		flip_v_(config["flip_vert"]),
		nostereo_(config["nostereo"]),
		downsize_(config.value("scale", 1.0f)) {
	// Use cameras
	camera_a_ = new VideoCapture((flip_) ? 1 : 0);
	if (!nostereo_) {
		camera_b_ = new VideoCapture((flip_) ? 0 : 1);
	} else {
		camera_b_ = nullptr;
	}

	if (!camera_a_->isOpened()) {
		delete camera_a_;
		if (camera_b_) delete camera_b_;
		camera_a_ = nullptr;
		camera_b_ = nullptr;
		LOG(FATAL) << "No cameras found";
		return;
	}

	if (!camera_b_ || !camera_b_->isOpened()) {
		if (camera_b_) delete camera_b_;
		camera_b_ = nullptr;
		stereo_ = false;
		LOG(WARNING) << "Not able to find second camera for stereo";
	} else {
		stereo_ = true;
	}
}

LocalSource::LocalSource(const string &vid, nlohmann::json &config)
	:	timestamp_(0.0),
		flip_(config["flip"]),
		flip_v_(config["flip_vert"]),
		nostereo_(config["nostereo"]),
		downsize_(config.value("scale", 1.0f)) {
	if (vid == "") {
		LOG(FATAL) << "No video file specified";
		camera_a_ = nullptr;
		camera_b_ = nullptr;
		return;
	}

	camera_a_ = new VideoCapture(vid.c_str());
	camera_b_ = nullptr;

	if (!camera_a_->isOpened()) {
		delete camera_a_;
		camera_a_ = nullptr;
		LOG(FATAL) << "Unable to load video file";
		return;
	}

	// Read first frame to determine stereo
	Mat frame;
	if (!camera_a_->read(frame)) {
		LOG(FATAL) << "No data in video file";
	}

	if (frame.cols >= 2*frame.rows) {
		LOG(INFO) << "Video size : " << frame.cols/2 << "x" << frame.rows;
		stereo_ = true;
	} else {
		LOG(INFO) << "Video size : " << frame.cols << "x" << frame.rows;
		stereo_ = false;
	}
}

bool LocalSource::left(cv::Mat &l) {
	if (!camera_a_) return false;

	if (!camera_a_->grab()) {
		LOG(ERROR) << "Unable to grab from camera A";
		return false;
	}

	// Record timestamp
	timestamp_ = duration_cast<duration<double>>(
			high_resolution_clock::now().time_since_epoch()).count();

	if (camera_b_ || !stereo_) {
		if (!camera_a_->retrieve(l)) {
			LOG(ERROR) << "Unable to read frame from camera A";
			return false;
		}
	} else {
		Mat frame;
		if (!camera_a_->retrieve(frame)) {
			LOG(ERROR) << "Unable to read frame from video";
			return false;
		}

		int resx = frame.cols / 2;
		if (flip_) {
			l = Mat(frame, Rect(resx, 0, frame.cols-resx, frame.rows));
		} else {
			l = Mat(frame, Rect(0, 0, resx, frame.rows));
		}
	}

	return true;
}

bool LocalSource::right(cv::Mat &r) {
	if (!camera_a_->grab()) {
		LOG(ERROR) << "Unable to grab from camera A";
		return false;
	}
	if (camera_b_ && !camera_b_->grab()) {
		LOG(ERROR) << "Unable to grab from camera B";
		return false;
	}

	// Record timestamp
	timestamp_ = duration_cast<duration<double>>(
			high_resolution_clock::now().time_since_epoch()).count();

	if (camera_b_ || !stereo_) {
		if (camera_b_ && !camera_b_->retrieve(r)) {
			LOG(ERROR) << "Unable to read frame from camera B";
			return false;
		}
	} else {
		Mat frame;
		if (!camera_a_) return false;
		if (!camera_a_->retrieve(frame)) {
			LOG(ERROR) << "Unable to read frame from video";
			return false;
		}

		int resx = frame.cols / 2;
		if (flip_) {
			r = Mat(frame, Rect(0, 0, resx, frame.rows));
		} else {
			r = Mat(frame, Rect(resx, 0, frame.cols-resx, frame.rows));
		}
	}

	return true;
}

bool LocalSource::get(cv::Mat &l, cv::Mat &r) {
	if (!camera_a_) return false;

	if (!camera_a_->grab()) {
		LOG(ERROR) << "Unable to grab from camera A";
		return false;
	}
	if (camera_b_ && !camera_b_->grab()) {
		LOG(ERROR) << "Unable to grab from camera B";
		return false;
	}

	// Record timestamp
	timestamp_ = duration_cast<duration<double>>(
			high_resolution_clock::now().time_since_epoch()).count();

	if (camera_b_ || !stereo_) {
		if (!camera_a_->retrieve(l)) {
			LOG(ERROR) << "Unable to read frame from camera A";
			return false;
		}
		if (camera_b_ && !camera_b_->retrieve(r)) {
			LOG(ERROR) << "Unable to read frame from camera B";
			return false;
		}
	} else {
		Mat frame;
		if (!camera_a_->retrieve(frame)) {
			LOG(ERROR) << "Unable to read frame from video";
			return false;
		}

		int resx = frame.cols / 2;
		if (flip_) {
			r = Mat(frame, Rect(0, 0, resx, frame.rows));
			l = Mat(frame, Rect(resx, 0, frame.cols-resx, frame.rows));
		} else {
			l = Mat(frame, Rect(0, 0, resx, frame.rows));
			r = Mat(frame, Rect(resx, 0, frame.cols-resx, frame.rows));
		}
	}

	if (downsize_ != 1.0f) {
		cv::resize(l, l, cv::Size(l.cols * downsize_, l.rows * downsize_),
				0, 0, cv::INTER_LINEAR);
		cv::resize(r, r, cv::Size(r.cols * downsize_, r.rows * downsize_),
				0, 0, cv::INTER_LINEAR);
	}

	if (flip_v_) {
		Mat tl, tr;
		cv::flip(l, tl, 0);
		cv::flip(r, tr, 0);
		l = tl;
		r = tr;
	}

	return true;
}

double LocalSource::getTimestamp() const {
	return timestamp_;
}

bool LocalSource::isStereo() const {
	return stereo_ && !nostereo_;
}
