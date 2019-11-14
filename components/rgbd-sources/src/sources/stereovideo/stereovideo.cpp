#include <loguru.hpp>

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

	cv::Size size = cv::Size(lsrc_->width(), lsrc_->height());
	frames_ = std::vector<Frame>(2);

	calib_ = ftl::create<Calibrate>(host_, "calibration", size, stream_);
	if (!calib_->isCalibrated()) LOG(WARNING) << "Cameras are not calibrated!";

	// Generate camera parameters from camera matrix
	cv::Mat K = calib_->getCameraMatrix();
	params_ = {
		K.at<double>(0,0),	// Fx
		K.at<double>(1,1),	// Fy
		-K.at<double>(0,2),	// Cx
		-K.at<double>(1,2),	// Cy
		(unsigned int) lsrc_->width(),
		(unsigned int) lsrc_->height(),
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
	// only left mask used (not used)
	cv::cuda::GpuMat mask_r_gpu(lsrc_->height(), lsrc_->width(), CV_8U, 255);
	cv::cuda::GpuMat mask_l_gpu(lsrc_->height(), lsrc_->width(), CV_8U, 255);
	calib_->rectifyStereo(mask_l_gpu, mask_r_gpu, stream_);
	stream_.waitForCompletion();
	cv::Mat mask_l;
	mask_l_gpu.download(mask_l);
	mask_l_ = (mask_l == 0);
	
	pipeline_input_ = ftl::config::create<ftl::operators::Graph>(host_, "input");
	#ifdef HAVE_OPTFLOW
	pipeline_input_->append<ftl::operators::NVOpticalFlow>("optflow");
	#endif

	pipeline_depth_ = ftl::config::create<ftl::operators::Graph>(host_, "disparity");
	pipeline_depth_->append<ftl::operators::FixstarsSGM>("algorithm");

	#ifdef HAVE_OPTFLOW
	pipeline_depth_->append<ftl::operators::OpticalFlowTemporalSmoothing>("optflow_filter");
	#endif
	pipeline_depth_->append<ftl::operators::DisparityBilateralFilter>("bilateral_filter");
	pipeline_depth_->append<ftl::operators::DisparityToDepth>("calculate_depth");
	pipeline_depth_->append<ftl::operators::ColourChannels>("colour");  // Convert BGR to BGRA
	pipeline_depth_->append<ftl::operators::Normals>("normals");  // Estimate surface normals
	pipeline_depth_->append<ftl::operators::CrossSupport>("cross");
	pipeline_depth_->append<ftl::operators::DiscontinuityMask>("discontinuity_mask");
	pipeline_depth_->append<ftl::operators::AggreMLS>("mls");  // Perform MLS (using smoothing channel)

	LOG(INFO) << "StereoVideo source ready...";
	ready_ = true;
}

ftl::rgbd::Camera StereoVideoSource::parameters(Channel chan) {
	if (chan == Channel::Right) {
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

bool StereoVideoSource::capture(int64_t ts) {
	timestamp_ = ts;
	lsrc_->grab();
	return true;
}

bool StereoVideoSource::retrieve() {
	auto &frame = frames_[0];
	frame.reset();
	auto &left = frame.create<cv::cuda::GpuMat>(Channel::Left);
	auto &right = frame.create<cv::cuda::GpuMat>(Channel::Right);
	lsrc_->get(left, right, calib_, stream2_);

	pipeline_input_->apply(frame, frame, host_, cv::cuda::StreamAccessor::getStream(stream2_));
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

	if (chan == Channel::Depth) {
		pipeline_depth_->apply(frame, frame, host_, cv::cuda::StreamAccessor::getStream(stream_));	
		stream_.waitForCompletion();
		host_->notify(timestamp_,
						frame.get<cv::cuda::GpuMat>(Channel::Left),
						frame.get<cv::cuda::GpuMat>(Channel::Depth));

	} else if (chan == Channel::Right) {
		stream_.waitForCompletion(); // TODO:(Nick) Move to getFrames
		host_->notify(timestamp_,
						frame.get<cv::cuda::GpuMat>(Channel::Left),
						frame.get<cv::cuda::GpuMat>(Channel::Right));
	
	} else {
		stream_.waitForCompletion(); // TODO:(Nick) Move to getFrames
		auto &left = frame.get<cv::cuda::GpuMat>(Channel::Left);
		depth_.create(left.size(), left.type());
		host_->notify(timestamp_,
						frame.get<cv::cuda::GpuMat>(Channel::Left),
						depth_);
	}

	return true;
}

bool StereoVideoSource::isReady() {
	return ready_;
}
