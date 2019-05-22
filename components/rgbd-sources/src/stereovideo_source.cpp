#include <glog/logging.h>
#include <ftl/stereovideo_source.hpp>
#include <ftl/configuration.hpp>
#include "calibrate.hpp"
#include "local.hpp"
#include "disparity.hpp"

using ftl::Calibrate;
using ftl::LocalSource;
using ftl::rgbd::StereoVideoSource;
using std::string;

StereoVideoSource::StereoVideoSource(nlohmann::json &config, ftl::net::Universe *net)
		: RGBDSource(config, net) {

}

StereoVideoSource::StereoVideoSource(nlohmann::json &config, const string &file)
		: RGBDSource(config), ready_(false) {

	REQUIRED({
		{"source","Details on source video [object]","object"}
	});
	
	if (ftl::is_video(file)) {
		// Load video file
		LOG(INFO) << "Using video file...";
		lsrc_ = new LocalSource(file, config["source"]);
	} else if (file != "") {
		auto vid = ftl::locateFile("video.mp4");
		if (!vid) {
			LOG(FATAL) << "No video.mp4 file found in provided paths";
		} else {
			LOG(INFO) << "Using test directory...";
			lsrc_ = new LocalSource(*vid, config["source"]);
		}
	} else {
		// Use cameras
		LOG(INFO) << "Using cameras...";
		lsrc_ = new LocalSource(config["source"]);
	}

	calib_ = new Calibrate(lsrc_, config["calibration"]);
	if (config["calibrate"]) calib_->recalibrate();
	if (!calib_->isCalibrated()) LOG(WARNING) << "Cameras are not calibrated!";
	else LOG(INFO) << "Calibration initiated.";

	// Generate camera parameters from Q matrix
	cv::Mat q = calib_->getCameraMatrix();
	params_ = {
		// TODO(Nick) Add fx and fy
		q.at<double>(0,0),	// F
		q.at<double>(0,2),	// Cx
		q.at<double>(1,2),	// Cy
		(unsigned int)left_.cols,  // TODO (Nick)
		(unsigned int)left_.rows,
		0.0f,	// 0m min
		15.0f	// 15m max
	};

	disp_ = Disparity::create(config["disparity"]);
    if (!disp_) LOG(FATAL) << "Unknown disparity algorithm : " << config["disparity"];

	LOG(INFO) << "StereoVideo source ready...";
	ready_ = true;
}

StereoVideoSource::~StereoVideoSource() {
	delete disp_;
	delete calib_;
	delete lsrc_;
}

void StereoVideoSource::grab() {
	calib_->rectified(left_, right_);
}

bool StereoVideoSource::isReady() {
	return ready_;
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
			dptr[x] = (homg_pt[2] / homg_pt[3]) / 1000.0f; // Depth in meters

			if( fabs(d) <= FLT_EPSILON )
				dptr[x] = 1000.0f;
		}
	}
}

void StereoVideoSource::getRGBD(cv::Mat &rgb, cv::Mat &depth) {
	cv::Mat disp;
	disp_->compute(left_, right_, disp);
	rgb = left_;
	disparityToDepth(disp, depth, calib_->getQ());
	//calib_->distort(rgb,depth);
}
