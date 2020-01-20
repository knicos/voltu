#include <loguru.hpp>

#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>

#include "stereovideo.hpp"

#include "ftl/configuration.hpp"

#ifdef HAVE_OPTFLOW
#include "ftl/operators/opticalflow.hpp"
#endif

#include "ftl/operators/smoothing.hpp"
#include "ftl/operators/colours.hpp"
#include "ftl/operators/normals.hpp"
#include "ftl/operators/filling.hpp"
#include "ftl/operators/segmentation.hpp"
#include "ftl/operators/disparity.hpp"
#include "ftl/operators/mask.hpp"

#include "ftl/threads.hpp"
#include "calibrate.hpp"
#include "local.hpp"
#include "disparity.hpp"

using ftl::rgbd::detail::Calibrate;
using ftl::rgbd::detail::LocalSource;
using ftl::rgbd::detail::StereoVideoSource;
using ftl::codecs::Channel;
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
	
	color_size_ = cv::Size(lsrc_->width(), lsrc_->height());
	frames_ = std::vector<Frame>(2);

	pipeline_input_ = ftl::config::create<ftl::operators::Graph>(host_, "input");
	#ifdef HAVE_OPTFLOW
	pipeline_input_->append<ftl::operators::NVOpticalFlow>("optflow");
	#endif

	calib_ = ftl::create<Calibrate>(host_, "calibration", cv::Size(lsrc_->fullWidth(), lsrc_->fullHeight()), stream_);

	string fname_default = "calibration.yml";
	auto fname_config = calib_->get<string>("calibration");
	string fname = fname_config ? *fname_config : fname_default;
	auto calibf = ftl::locateFile(fname);
	if (calibf) {
		fname = *calibf;
		if (calib_->loadCalibration(fname)) {
			calib_->calculateRectificationParameters();
			calib_->setRectify(true);
		}
	}
	else {
		fname = fname_config ? *fname_config : 
								string(FTL_LOCAL_CONFIG_ROOT) + "/"
								+ std::string("calibration.yml");
		
		LOG(ERROR) << "No calibration, default path set to " + fname;
	}

	////////////////////////////////////////////////////////////////////////////
	// RPC callbacks to update calibration
	// Should only be used by calibration app (interface may change)
	// Tries to follow interface of ftl::Calibrate
	
	host_->getNet()->bind("set_pose",
		[this](std::vector<double> data){
			if (data.size() != 16) {
				LOG(ERROR) << "invalid pose received (wrong size)";
				return false;
			}

			cv::Mat M = cv::Mat(data).reshape(1, 4).t();
			if (!calib_->setPose(M)) {
				LOG(ERROR) << "invalid pose received (bad value)";
				return false;
			}

			return true;
	});

	host_->getNet()->bind("set_intrinsics",
		[this](	std::vector<int> size,
				std::vector<double> camera_l, std::vector<double> d_left,
				std::vector<double> camera_r, std::vector<double> d_right) {
		if ((size.size() != 2) || (camera_l.size() != 9) || (camera_r.size() != 9)) {
			LOG(ERROR) << "bad intrinsic parameters (wrong size)";
			return false;
		}
		cv::Size calib_size(size[0], size[1]);
		cv::Mat K_l = cv::Mat(camera_l).reshape(1, 3).t();
		cv::Mat K_r = cv::Mat(camera_r).reshape(1, 3).t();
		cv::Mat D_l = cv::Mat(D_l);
		cv::Mat D_r = cv::Mat(D_r);
		
		if (!calib_->setIntrinsics(calib_size, {K_l, K_r}, {D_l, D_r})) {
			LOG(ERROR) << "bad intrinsic parameters (bad values)";
			return false;
		}
		return true;
	});

	host_->getNet()->bind("set_extrinsics",
		[this](std::vector<double> data_rvec, std::vector<double> data_tvec){
			if ((data_rvec.size() != 3) || (data_tvec.size() != 3)) {
				LOG(ERROR) << "invalid extrinsic parameters received (wrong size)";
				return false;
			}

			cv::Mat R;
			cv::Rodrigues(data_rvec, R);
			cv::Mat t(data_tvec);
			
			if (!calib_->setExtrinsics(R, t)) {
				LOG(ERROR) << "invalid extrinsic parameters (bad values)";
				return false;
			}
			return true;
	});

	host_->getNet()->bind("save_calibration", 
		[this, fname](){
			return calib_->saveCalibration(fname);
	});

	host_->getNet()->bind("use_rectify", 
		[this](bool enable){
			bool retval = enable && calib_->setRectify(enable);
			updateParameters();
			return retval;
	});

	////////////////////////////////////////////////////////////////////////////

	// Generate camera parameters from camera matrix
	updateParameters();
	
	LOG(INFO) << "StereoVideo source ready...";
	ready_ = true;

	state_.set("name", host_->value("name", host_->getID()));
}

ftl::rgbd::Camera StereoVideoSource::parameters(Channel chan) {
	if (chan == Channel::Right) {
		return state_.getRight();
	} else {
		return state_.getLeft();
	}
}

void StereoVideoSource::updateParameters() {
	Eigen::Matrix4d pose;
	cv::cv2eigen(calib_->getPose(), pose);
	setPose(pose);

	cv::Mat K;
	
	// same for left and right
	double baseline = 1.0 / calib_->getQ().at<double>(3,2);
	double doff =  -calib_->getQ().at<double>(3,3) * baseline;

	// left

	K = calib_->getCameraMatrixLeft(color_size_);
	state_.getLeft() = {
		K.at<double>(0,0),	// Fx
		K.at<double>(1,1),	// Fy
		-K.at<double>(0,2),	// Cx
		-K.at<double>(1,2),	// Cy
		(unsigned int) color_size_.width,
		(unsigned int) color_size_.height,
		0.0f,	// 0m min
		15.0f,	// 15m max
		baseline, // Baseline
		doff
	};
	
	host_->getConfig()["focal"] = params_.fx;
	host_->getConfig()["centre_x"] = params_.cx;
	host_->getConfig()["centre_y"] = params_.cy;
	host_->getConfig()["baseline"] = params_.baseline;
	host_->getConfig()["doffs"] = params_.doffs;

	// right

	K = calib_->getCameraMatrixRight(color_size_);
	state_.getRight() = {
		K.at<double>(0,0),	// Fx
		K.at<double>(1,1),	// Fy
		-K.at<double>(0,2),	// Cx
		-K.at<double>(1,2),	// Cy
		(unsigned int) color_size_.width,
		(unsigned int) color_size_.height,
		0.0f,	// 0m min
		15.0f,	// 15m max
		baseline, // Baseline
		doff
	};
}

bool StereoVideoSource::capture(int64_t ts) {
	timestamp_ = ts;
	lsrc_->grab();
	return true;
}

bool StereoVideoSource::retrieve() {
	auto &frame = frames_[0];
	frame.reset();
	frame.setOrigin(&state_);
	auto &left = frame.create<cv::cuda::GpuMat>(Channel::Left);
	auto &right = frame.create<cv::cuda::GpuMat>(Channel::Right);
	cv::cuda::GpuMat dummy;
	auto &hres = (lsrc_->hasHigherRes()) ? frame.create<cv::cuda::GpuMat>(Channel::ColourHighRes) : dummy;

	lsrc_->get(left, right, hres, calib_, stream2_);

	//LOG(INFO) << "Channel size: " << hres.size();

	pipeline_input_->apply(frame, frame, cv::cuda::StreamAccessor::getStream(stream2_));
	stream2_.waitForCompletion();
	
	return true;
}

void StereoVideoSource::swap() {
	auto tmp = std::move(frames_[0]);
	frames_[0] = std::move(frames_[1]);
	frames_[1] = std::move(tmp);
}

bool StereoVideoSource::compute(int n, int b) {
	auto &frame = frames_[1];
	
	const ftl::codecs::Channel chan = host_->getChannel();
	if (!frame.hasChannel(Channel::Left) || !frame.hasChannel(Channel::Right)) {
		return false;
	}

	cv::cuda::GpuMat& left = frame.get<cv::cuda::GpuMat>(Channel::Left);
	cv::cuda::GpuMat& right = frame.get<cv::cuda::GpuMat>(Channel::Right);

	if (left.empty() || right.empty()) {
		return false;
	}

	//stream_.waitForCompletion();
	host_->notify(timestamp_, frame);

	return true;
}

bool StereoVideoSource::isReady() {
	return ready_;
}
