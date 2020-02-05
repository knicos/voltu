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

#include <ftl/timer.hpp>

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
	}
	else {
		camera_b_->set(cv::CAP_PROP_FRAME_WIDTH, value("width", 640));
		camera_b_->set(cv::CAP_PROP_FRAME_HEIGHT, value("height", 480));
		camera_b_->set(cv::CAP_PROP_FPS, 1000 / ftl::timer::getInterval());
		//camera_b_->set(cv::CAP_PROP_BUFFERSIZE, 0);  // Has no effect

		stereo_ = true;
	}

	camera_a_->set(cv::CAP_PROP_FRAME_WIDTH, value("width", 640));
	camera_a_->set(cv::CAP_PROP_FRAME_HEIGHT, value("height", 480));
	camera_a_->set(cv::CAP_PROP_FPS, 1000 / ftl::timer::getInterval());
	//camera_a_->set(cv::CAP_PROP_BUFFERSIZE, 0);  // Has no effect
	
	Mat frame;
	camera_a_->grab();
	camera_a_->retrieve(frame);
	LOG(INFO) << "Video size : " << frame.cols << "x" << frame.rows;
	width_ = frame.cols;
	height_ = frame.rows;

	dwidth_ = value("depth_width", width_);
	dheight_ = value("depth_height", height_);

	// Allocate page locked host memory for fast GPU transfer
	left_hm_ = cv::cuda::HostMem(dheight_, dwidth_, CV_8UC3);
	right_hm_ = cv::cuda::HostMem(dheight_, dwidth_, CV_8UC3);
	hres_hm_ = cv::cuda::HostMem(height_, width_, CV_8UC3);
}

LocalSource::LocalSource(nlohmann::json &config, const string &vid)
	:	Configurable(config), timestamp_(0.0) {
	LOG(FATAL) << "Stereo video file sources no longer supported";
 /*
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

	dwidth_ = value("depth_width", width_);
	dheight_ = value("depth_height", height_);

	// Allocate page locked host memory for fast GPU transfer
	left_hm_ = cv::cuda::HostMem(dheight_, dwidth_, CV_8UC3);
	right_hm_ = cv::cuda::HostMem(dheight_, dwidth_, CV_8UC3);
	hres_hm_ = cv::cuda::HostMem(height_, width_, CV_8UC3);

	//tps_ = 1.0 / value("max_fps", 25.0);
	*/
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

	return true;
}

bool LocalSource::get(cv::cuda::GpuMat &l_out, cv::cuda::GpuMat &r_out, cv::cuda::GpuMat &hres_out, Calibrate *c, cv::cuda::Stream &stream) {
	Mat l, r ,hres;

	// Use page locked memory
	l = left_hm_.createMatHeader();
	r = right_hm_.createMatHeader();
	hres = hres_hm_.createMatHeader();

	Mat &lfull = (!hasHigherRes()) ? l : hres;
	Mat &rfull = (!hasHigherRes()) ? r : rtmp_;

	if (!camera_a_) return false;

	// TODO: Use threads here?
	if (!camera_a_->retrieve(lfull)) {
		LOG(ERROR) << "Unable to read frame from camera A";
		return false;
	}

	if (camera_b_ && !camera_b_->retrieve(rfull)) {
		LOG(ERROR) << "Unable to read frame from camera B";
		return false;
	}

	if (stereo_) {
		c->rectifyStereo(lfull, rfull);
		
		// Need to resize
		if (hasHigherRes()) {
			// TODO: Use threads?
			cv::resize(rfull, r, r.size(), 0.0, 0.0, cv::INTER_CUBIC);
		}
	}

	if (hasHigherRes()) {
		cv::resize(lfull, l, l.size(), 0.0, 0.0, cv::INTER_CUBIC);
		hres_out.upload(hres, stream);
	} else {
		hres_out = cv::cuda::GpuMat();
	}

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

