#include <ftl/local.hpp>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>
#include <glog/logging.h>

using ftl::LocalSource;
using cv::Mat;
using cv::VideoCapture;
using cv::Rect;
using std::string;
using namespace std::chrono;

LocalSource::LocalSource() : timestamp_(0.0) {
	// Use cameras
	camera_a_ = new VideoCapture(0);
	camera_b_ = new VideoCapture(1);

	if (!camera_a_->isOpened()) {
		delete camera_a_;
		delete camera_b_;
		camera_a_ = nullptr;
		camera_b_ = nullptr;
		LOG(FATAL) << "No cameras found";
		return;
	}

	if (!camera_b_->isOpened()) {
		delete camera_b_;
		camera_b_ = nullptr;
		stereo_ = false;
		LOG(WARNING) << "Not able to find second camera for stereo";
	} else {
		stereo_ = true;
	}
}

LocalSource::LocalSource(const string &vid): timestamp_(0.0) {
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
		stereo_ = true;
	} else {
		stereo_ = false;
	}
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
		l = Mat(frame, Rect(0,0,resx,frame.rows));
		r = Mat(frame, Rect(resx,0,frame.cols,frame.rows));
	}
	
	return true;
}

double LocalSource::getTimestamp() const {
	return timestamp_;
}
	
bool LocalSource::isStereo() const {
	return stereo_;
}

