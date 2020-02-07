/*
 * Copyright 2019 Nicolas Pope
 */

#include <loguru.hpp>

#include <string>
#include <chrono>
#include <ftl/threads.hpp>
#include <ftl/profiler.hpp>

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

	LOG(INFO) << "Video backend: " << camera_a_->getBackendName();
	LOG(INFO) << "Video defaults: " << camera_a_->get(cv::CAP_PROP_FRAME_WIDTH) << "x" << camera_a_->get(cv::CAP_PROP_FRAME_HEIGHT) << " @ " << camera_a_->get(cv::CAP_PROP_FPS);

	camera_a_->set(cv::CAP_PROP_FRAME_WIDTH, value("width", 640));
	camera_a_->set(cv::CAP_PROP_FRAME_HEIGHT, value("height", 480));
	camera_a_->set(cv::CAP_PROP_FPS, 1000 / ftl::timer::getInterval());
	//camera_a_->set(cv::CAP_PROP_BUFFERSIZE, 0);  // Has no effect
	
	Mat frame;
	if (!camera_a_->grab()) LOG(ERROR) << "Could not grab a video frame";
	camera_a_->retrieve(frame);
	LOG(INFO) << "Video size : " << frame.cols << "x" << frame.rows;
	width_ = frame.cols;
	height_ = frame.rows;

	dwidth_ = value("depth_width", width_);
	dheight_ = value("depth_height", height_);

	// Allocate page locked host memory for fast GPU transfer
	left_hm_ = cv::cuda::HostMem(dheight_, dwidth_, CV_8UC4);
	right_hm_ = cv::cuda::HostMem(dheight_, dwidth_, CV_8UC4);
	hres_hm_ = cv::cuda::HostMem(height_, width_, CV_8UC4);
}

LocalSource::LocalSource(nlohmann::json &config, const string &vid)
	:	Configurable(config), timestamp_(0.0) {
	LOG(FATAL) << "Stereo video file sources no longer supported";
}


bool LocalSource::grab() {
	if (!camera_a_) return false;

	if (camera_b_) {
		if (!camera_a_->grab()) {
			LOG(ERROR) << "Unable to grab from camera A";
			return false;
		}
		if (camera_b_ && !camera_b_->grab()) {
			LOG(ERROR) << "Unable to grab from camera B";
			return false;
		}
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

	std::future<bool> future_b;
	if (camera_b_) {
		future_b = std::move(ftl::pool.push([this,&rfull,&r,c,&r_out,&stream](int id) {
			if (!camera_b_->retrieve(frame_r_)) {
				LOG(ERROR) << "Unable to read frame from camera B";
				return false;
			}

			cv::cvtColor(frame_r_, rfull, cv::COLOR_BGR2BGRA);

			if (stereo_) {
				c->rectifyRight(rfull);

				if (hasHigherRes()) {
					// TODO: Use threads?
					cv::resize(rfull, r, r.size(), 0.0, 0.0, cv::INTER_CUBIC);
				}
			}

			r_out.upload(r, stream);
			return true;
		}));
	}

	if (camera_b_) {
		//FTL_Profile("Camera Retrieve", 0.01);
		// TODO: Use threads here?
		if (!camera_a_->retrieve(frame_l_)) {
			LOG(ERROR) << "Unable to read frame from camera A";
			return false;
		}

		/*if (camera_b_ && !camera_b_->retrieve(rfull)) {
			LOG(ERROR) << "Unable to read frame from camera B";
			return false;
		}*/
	} else {
		if (!camera_a_->read(frame_l_)) {
			LOG(ERROR) << "Unable to read frame from camera A";
			return false;
		}
	}

	cv::cvtColor(frame_l_, lfull, cv::COLOR_BGR2BGRA);

	if (stereo_) {
		//FTL_Profile("Rectification", 0.01);
		//c->rectifyStereo(lfull, rfull);
		c->rectifyLeft(lfull);
		
		// Need to resize
		//if (hasHigherRes()) {
			// TODO: Use threads?
		//	cv::resize(rfull, r, r.size(), 0.0, 0.0, cv::INTER_CUBIC);
		//}
	}

	if (hasHigherRes()) {
		//FTL_Profile("Frame Resize", 0.01);
		cv::resize(lfull, l, l.size(), 0.0, 0.0, cv::INTER_CUBIC);
		hres_out.upload(hres, stream);
	} else {
		hres_out = cv::cuda::GpuMat();
	}

	{
		//FTL_Profile("Upload", 0.05);
		l_out.upload(l, stream);
	}
	//r_out.upload(r, stream);

	if (camera_b_) {
		//FTL_Profile("WaitCamB", 0.05);
		future_b.wait();
	}

	return true;
}

double LocalSource::getTimestamp() const {
	return timestamp_;
}

bool LocalSource::isStereo() const {
	return stereo_ && !nostereo_;
}

