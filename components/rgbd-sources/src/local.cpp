/*
 * Copyright 2019 Nicolas Pope
 */

#include <loguru.hpp>

#include <string>
#include <chrono>
#include <thread>

#include "local.hpp"
#include "calibrate.hpp"
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xphoto.hpp>

using ftl::rgbd::detail::LocalSource;
using ftl::rgbd::detail::Calibrate;
using cv::Mat;
using cv::VideoCapture;
using cv::Rect;
using std::string;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;
using std::this_thread::sleep_for;

LocalSource::LocalSource(nlohmann::json &config)
		: Configurable(config), timestamp_(0.0) {

	nostereo_ = value("nostereo", false);

	// Use cameras
	camera_a_ = new VideoCapture;
	LOG(INFO) << "Cameras check... ";
	camera_a_->open(0);

	if (!nostereo_) {
		camera_b_ = new VideoCapture(1);
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
		camera_a_->set(cv::CAP_PROP_FRAME_WIDTH, value("width", 640));
		camera_a_->set(cv::CAP_PROP_FRAME_HEIGHT, value("height", 480));
		camera_b_->set(cv::CAP_PROP_FRAME_WIDTH, value("width", 640));
		camera_b_->set(cv::CAP_PROP_FRAME_HEIGHT, value("height", 480));

		Mat frame;
		camera_a_->grab();
		camera_a_->retrieve(frame);
		LOG(INFO) << "Video size : " << frame.cols << "x" << frame.rows;
		width_ = frame.cols;
		height_ = frame.rows;
		stereo_ = true;
	}

	// Allocate page locked host memory for fast GPU transfer
	left_hm_ = cv::cuda::HostMem(height_, width_, CV_8UC3);
	right_hm_ = cv::cuda::HostMem(height_, width_, CV_8UC3);
}

LocalSource::LocalSource(nlohmann::json &config, const string &vid)
	:	Configurable(config), timestamp_(0.0) {

	//flip_ = value("flip", false);
	//flip_v_ = value("flip_vert", false);
	nostereo_ = value("nostereo", false);
	//downsize_ = value("scale", 1.0f);

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
		width_ = frame.cols / 2;
		height_ = frame.rows;
		stereo_ = true;
	} else {
		LOG(INFO) << "Video size : " << frame.cols << "x" << frame.rows;
		width_ = frame.cols;
		height_ = frame.rows;
		stereo_ = false;
	}

	// Allocate page locked host memory for fast GPU transfer
	left_hm_ = cv::cuda::HostMem(height_, width_, CV_8UC3);
	right_hm_ = cv::cuda::HostMem(height_, width_, CV_8UC3);

	//tps_ = 1.0 / value("max_fps", 25.0);
}

/*bool LocalSource::left(cv::Mat &l) {
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
}*/

/*bool LocalSource::right(cv::Mat &r) {
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
}*/

bool LocalSource::grab() {
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
	double timestamp = duration_cast<duration<double>>(
			high_resolution_clock::now().time_since_epoch()).count();
	
	// Limit max framerate
	//if (timestamp - timestamp_ < tps_) {
	//	sleep_for(milliseconds((int)std::round((tps_ - (timestamp - timestamp_))*1000)));
	//}

	timestamp_ = timestamp;

	return true;
}

bool LocalSource::get(cv::cuda::GpuMat &l_out, cv::cuda::GpuMat &r_out, Calibrate *c, cv::cuda::Stream &stream) {
	Mat l, r;

	// Use page locked memory
	l = left_hm_.createMatHeader();
	r = right_hm_.createMatHeader();

	if (!camera_a_) return false;

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
		//if (flip_) {
		//	r = Mat(frame, Rect(0, 0, resx, frame.rows));
		//	l = Mat(frame, Rect(resx, 0, frame.cols-resx, frame.rows));
		//} else {
			l = Mat(frame, Rect(0, 0, resx, frame.rows));
			r = Mat(frame, Rect(resx, 0, frame.cols-resx, frame.rows));
		//}
	}

	/*if (downsize_ != 1.0f) {
		// cv::cuda::resize()

		cv::resize(left_, left_, cv::Size((int)(left_.cols * downsize_), (int)(left_.rows * downsize_)),
				0, 0, cv::INTER_LINEAR);
		cv::resize(r, r, cv::Size((int)(r.cols * downsize_), (int)(r.rows * downsize_)),
				0, 0, cv::INTER_LINEAR);
	}*/

	// Note: this seems to be too slow on CPU...
	/*cv::Ptr<cv::xphoto::WhiteBalancer> wb;
	wb = cv::xphoto::createSimpleWB();
	wb->balanceWhite(l, l);
	wb->balanceWhite(r, r);*/

	/*if (flip_v_) {
		Mat tl, tr;
		cv::flip(left_, tl, 0);
		cv::flip(r, tr, 0);
		left_ = tl;
		r = tr;
	}*/

	c->rectifyStereo(l, r);

	l_out.upload(l, stream);
	r_out.upload(r, stream);

	return true;
}

double LocalSource::getTimestamp() const {
	return timestamp_;
}

bool LocalSource::isStereo() const {
	return stereo_ && !nostereo_;
}

