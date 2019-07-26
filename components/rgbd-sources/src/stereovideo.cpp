#include <loguru.hpp>
#include "stereovideo.hpp"
#include <ftl/configuration.hpp>
#include <ftl/threads.hpp>
#include "calibrate.hpp"
#include "local.hpp"
#include "disparity.hpp"
#include "cuda_algorithms.hpp"

using ftl::rgbd::detail::Calibrate;
using ftl::rgbd::detail::LocalSource;
using ftl::rgbd::detail::StereoVideoSource;
using std::string;

StereoVideoSource::StereoVideoSource(ftl::rgbd::Source *host)
		: ftl::rgbd::detail::Source(host), ready_(false) {
	init("");
}

StereoVideoSource::StereoVideoSource(ftl::rgbd::Source *host, const string &file)
		: ftl::rgbd::detail::Source(host), ready_(false) {

	init(file);
}

StereoVideoSource::~StereoVideoSource() {
	delete disp_;
	delete calib_;
	delete lsrc_;
}

void StereoVideoSource::init(const string &file) {
	capabilities_ = kCapVideo | kCapStereo;

	if (ftl::is_video(file)) {
		// Load video file
		LOG(INFO) << "Using video file...";
		lsrc_ = ftl::create<LocalSource>(host_, "feed", file);
	} else if (ftl::is_directory(file)) {
		// FIXME: This is not an ideal solution...
		ftl::config::addPath(file);

		auto vid = ftl::locateFile("video.mp4");
		if (!vid) {
			LOG(FATAL) << "No video.mp4 file found in provided paths (" << file << ")";
		} else {
			LOG(INFO) << "Using test directory...";
			lsrc_ = ftl::create<LocalSource>(host_, "feed", *vid);
		}
	}
	else {
		// Use cameras
		LOG(INFO) << "Using cameras...";
		lsrc_ = ftl::create<LocalSource>(host_, "feed");
	}

	cv::Size size = cv::Size(lsrc_->width(), lsrc_->height());
	calib_ = ftl::create<Calibrate>(host_, "calibration", size, stream_);

	if (!calib_->isCalibrated()) LOG(WARNING) << "Cameras are not calibrated!";

	// Generate camera parameters from camera matrix
	cv::Mat q = calib_->getCameraMatrix();
	params_ = {
		q.at<double>(0,0),	// Fx
		q.at<double>(1,1),	// Fy
		-q.at<double>(0,2),	// Cx
		-q.at<double>(1,2),	// Cy
		(unsigned int)lsrc_->width(),
		(unsigned int)lsrc_->height(),
		0.0f,	// 0m min
		15.0f,	// 15m max
		1.0 / calib_->getQ().at<double>(3,2), // Baseline
		0.0f  // doffs
	};
	params_.doffs = -calib_->getQ().at<double>(3,3) * params_.baseline;

	// Add calibration to config object
	host_->getConfig()["focal"] = params_.fx;
	host_->getConfig()["centre_x"] = params_.cx;
	host_->getConfig()["centre_y"] = params_.cy;
	host_->getConfig()["baseline"] = params_.baseline;
	host_->getConfig()["doffs"] = params_.doffs;

	// Add event handlers to allow calibration changes...
	host_->on("baseline", [this](const ftl::config::Event &e) {
		params_.baseline = host_->value("baseline", params_.baseline);
		UNIQUE_LOCK(host_->mutex(), lk);
		calib_->updateCalibration(params_);
	});

	host_->on("focal", [this](const ftl::config::Event &e) {
		params_.fx = host_->value("focal", params_.fx);
		params_.fy = params_.fx;
		UNIQUE_LOCK(host_->mutex(), lk);
		calib_->updateCalibration(params_);
	});

	host_->on("doffs", [this](const ftl::config::Event &e) {
		params_.doffs = host_->value("doffs", params_.doffs);
	});
	
	// left and right masks (areas outside rectified images)
	// only left mask used
	cv::cuda::GpuMat mask_r_gpu(lsrc_->height(), lsrc_->width(), CV_8U, 255);
	cv::cuda::GpuMat mask_l_gpu(lsrc_->height(), lsrc_->width(), CV_8U, 255);
	
	calib_->rectifyStereo(mask_l_gpu, mask_r_gpu, stream_);
	stream_.waitForCompletion();

	cv::Mat mask_l;
	mask_l_gpu.download(mask_l);
	mask_l_ = (mask_l == 0);
	
	disp_ = Disparity::create(host_, "disparity");
    if (!disp_) LOG(FATAL) << "Unknown disparity algorithm : " << *host_->get<ftl::config::json_t>("disparity");
	disp_->setMask(mask_l_);

	LOG(INFO) << "StereoVideo source ready...";
	ready_ = true;
}

ftl::rgbd::Camera StereoVideoSource::parameters(ftl::rgbd::channel_t chan) {
	if (chan == ftl::rgbd::kChanRight) {
		cv::Mat q = calib_->getCameraMatrixRight();
		ftl::rgbd::Camera params = {
			q.at<double>(0,0),	// Fx
			q.at<double>(1,1),	// Fy
			-q.at<double>(0,2),	// Cx
			-q.at<double>(1,2),	// Cy
			(unsigned int)lsrc_->width(),
			(unsigned int)lsrc_->height(),
			0.0f,	// 0m min
			15.0f,	// 15m max
			1.0 / calib_->getQ().at<double>(3,2), // Baseline
			0.0f  // doffs
		};
		return params;
		//params_.doffs = -calib_->getQ().at<double>(3,3) * params_.baseline;
	} else {
		return params_;
	}
}

static void disparityToDepth(const cv::cuda::GpuMat &disparity, cv::cuda::GpuMat &depth,
							 const cv::Mat &Q, cv::cuda::Stream &stream) {
	// Q(3, 2) = -1/Tx
	// Q(2, 3) = f

	double val = (1.0f / Q.at<double>(3, 2)) * Q.at<double>(2, 3);
	cv::cuda::divide(val, disparity, depth, 1.0f / 1000.0f, -1, stream);
}

bool StereoVideoSource::grab(int n, int b) {
	const ftl::rgbd::channel_t chan = host_->getChannel();

	if (chan == ftl::rgbd::kChanDepth) {
		lsrc_->get(left_, right_, stream_);
		if (depth_tmp_.empty()) depth_tmp_ = cv::cuda::GpuMat(left_.size(), CV_32FC1);
		if (disp_tmp_.empty()) disp_tmp_ = cv::cuda::GpuMat(left_.size(), CV_32FC1);
		calib_->rectifyStereo(left_, right_, stream_);
		disp_->compute(left_, right_, disp_tmp_, stream_);
		ftl::cuda::disparity_to_depth(disp_tmp_, depth_tmp_, params_, stream_);
		left_.download(rgb_, stream_);
		//rgb_ = lsrc_->cachedLeft();
		depth_tmp_.download(depth_, stream_);

		stream_.waitForCompletion();  // TODO:(Nick) Move to getFrames
	} else if (chan == ftl::rgbd::kChanRight) {
		lsrc_->get(left_, right_, stream_);
		calib_->rectifyStereo(left_, right_, stream_);
		left_.download(rgb_, stream_);
		right_.download(depth_, stream_);
		stream_.waitForCompletion();  // TODO:(Nick) Move to getFrames
	} else {
		lsrc_->get(left_, right_, stream_);
		calib_->rectifyStereo(left_, right_, stream_);
		//rgb_ = lsrc_->cachedLeft();
		left_.download(rgb_, stream_);
		stream_.waitForCompletion();  // TODO:(Nick) Move to getFrames
	}
	return true;
}

bool StereoVideoSource::isReady() {
	return ready_;
}
