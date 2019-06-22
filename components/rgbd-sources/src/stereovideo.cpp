#include <loguru.hpp>
#include "stereovideo.hpp"
#include <ftl/configuration.hpp>
#include "calibrate.hpp"
#include "local.hpp"
#include "disparity.hpp"
#include <mutex>

using ftl::rgbd::detail::Calibrate;
using ftl::rgbd::detail::LocalSource;
using ftl::rgbd::detail::StereoVideoSource;
using std::string;
using std::mutex;
using std::unique_lock;

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
	LOG(INFO) << "STEREOSOURCE = " << file;
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
		15.0f	// 15m max
	};
	
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

static void disparityToDepth(const cv::cuda::GpuMat &disparity, cv::cuda::GpuMat &depth,
							 const cv::Mat &Q, cv::cuda::Stream &stream) {
	// Q(3, 2) = -1/Tx
	// Q(2, 3) = f

	double val = (1.0f / Q.at<double>(3, 2)) * Q.at<double>(2, 3);
	cv::cuda::divide(val, disparity, depth, 1.0f / 1000.0f, -1, stream);
}

bool StereoVideoSource::grab() {
	lsrc_->get(left_, right_, stream_);
	if (depth_tmp_.empty()) depth_tmp_ = cv::cuda::GpuMat(left_.size(), CV_32FC1);
	if (disp_tmp_.empty()) disp_tmp_ = cv::cuda::GpuMat(left_.size(), CV_32FC1);
	calib_->rectifyStereo(left_, right_, stream_);
	disp_->compute(left_, right_, disp_tmp_, stream_);
	disparityToDepth(disp_tmp_, depth_tmp_, calib_->getQ(), stream_);
	//left_.download(rgb_, stream_);
	rgb_ = lsrc_->cachedLeft();
	depth_tmp_.download(depth_, stream_);

	stream_.waitForCompletion();	
	return true;
}

bool StereoVideoSource::isReady() {
	return ready_;
}
