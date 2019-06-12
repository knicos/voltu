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
	if (ftl::is_video(file)) {
		// Load video file
		LOG(INFO) << "Using video file...";
		//lsrc_ = new LocalSource(file, config["source"]);
		lsrc_ = ftl::create<LocalSource>(host_, "feed", file);
	} else if (file != "") {
		auto vid = ftl::locateFile("video.mp4");
		if (!vid) {
			LOG(FATAL) << "No video.mp4 file found in provided paths (" << file << ")";
		} else {
			LOG(INFO) << "Using test directory...";
			//lsrc_ = new LocalSource(*vid, config["source"]);
			lsrc_ = ftl::create<LocalSource>(host_, "feed", *vid);
		}
	} else {
		// Use cameras
		LOG(INFO) << "Using cameras...";
		//lsrc_ = new LocalSource(config["source"]);
		lsrc_ = ftl::create<LocalSource>(host_, "feed");
	}

	//calib_ = new Calibrate(lsrc_, ftl::resolve(config["calibration"]));
	calib_ = ftl::create<Calibrate>(host_, "calibration", lsrc_);

	if (host_->value("calibrate", false)) calib_->recalibrate();
	if (!calib_->isCalibrated()) LOG(WARNING) << "Cameras are not calibrated!";
	else LOG(INFO) << "Calibration initiated.";

	// Generate camera parameters from Q matrix
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
	cv::Mat mask_r(lsrc_->height(), lsrc_->width(), CV_8U, 255);
	cv::Mat mask_l(lsrc_->height(), lsrc_->width(), CV_8U, 255);
	calib_->rectifyStereo(mask_l, mask_r);
	mask_l_ = (mask_l == 0);
	
	disp_ = Disparity::create(host_, "disparity");
    if (!disp_) LOG(FATAL) << "Unknown disparity algorithm : " << *host_->get<ftl::config::json_t>("disparity");
	disp_->setMask(mask_l_);

	LOG(INFO) << "StereoVideo source ready...";
	ready_ = true;
}

static void disparityToDepth(const cv::Mat &disparity, cv::Mat &depth, const cv::Mat &q) {
	cv::Matx44d _Q;
    q.convertTo(_Q, CV_64F);

	if (depth.empty()) depth = cv::Mat(disparity.size(), CV_32F);

	for( int y = 0; y < disparity.rows; y++ ) {
		const float *sptr = disparity.ptr<float>(y);
		float *dptr = depth.ptr<float>(y);

		for( int x = 0; x < disparity.cols; x++ ) {
			double d = sptr[x];
			cv::Vec4d homg_pt = _Q*cv::Vec4d(x, y, d, 1.0);
			//dptr[x] = Vec3d(homg_pt.val);
			//dptr[x] /= homg_pt[3];
			dptr[x] = (float)(homg_pt[2] / homg_pt[3]) / 1000.0f; // Depth in meters

			if( fabs(d) <= FLT_EPSILON )
				dptr[x] = 1000.0f;
		}
	}
}

bool StereoVideoSource::grab() {
	calib_->rectified(left_, right_);

	cv::Mat disp;
	disp_->compute(left_, right_, disp);

	//unique_lock<mutex> lk(mutex_);
	left_.copyTo(rgb_);
	disparityToDepth(disp, depth_, calib_->getQ());
	return true;
}

bool StereoVideoSource::isReady() {
	return ready_;
}