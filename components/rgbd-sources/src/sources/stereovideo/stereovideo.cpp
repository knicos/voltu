#include <loguru.hpp>
#include "stereovideo.hpp"

#include <ftl/configuration.hpp>

#ifdef HAVE_OPTFLOW
#include <ftl/operators/opticalflow.hpp>
#endif

#include <ftl/threads.hpp>
#include "calibrate.hpp"
#include "local.hpp"
#include "disparity.hpp"
#include "cuda_algorithms.hpp"

#include "cuda_algorithms.hpp"

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
	delete disp_;
	delete calib_;
	delete lsrc_;
}

void StereoVideoSource::init(const string &file)
{
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

	// Create the source depth map pipeline
	pipeline_ = ftl::config::create<ftl::operators::Graph>(host_, "disparity_pipeline");
	/*pipeline1->append<ftl::operators::ColourChannels>("colour");  // Convert BGR to BGRA
	pipeline1->append<ftl::operators::HFSmoother>("hfnoise");  // Remove high-frequency noise
	pipeline1->append<ftl::operators::Normals>("normals");  // Estimate surface normals
	pipeline1->append<ftl::operators::SmoothChannel>("smoothing");  // Generate a smoothing channel
	//pipeline1->append<ftl::operators::ScanFieldFill>("filling");  // Generate a smoothing channel
	pipeline1->append<ftl::operators::ColourMLS>("mls");  // Perform MLS (using smoothing channel)
	*/

	cv::Size size = cv::Size(lsrc_->width(), lsrc_->height());
	frames_ = std::vector<Frame>(2);

#ifdef HAVE_OPTFLOW

	use_optflow_ =  host_->value("use_optflow", false);
	LOG(INFO) << "Using optical flow: " << (use_optflow_ ? "true" : "false");
	pipeline_->append<ftl::operators::NVOpticalFlow>("optflow");

#endif

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

static void disparityToDepth(const cv::cuda::GpuMat &disparity, cv::cuda::GpuMat &depth,
							 const cv::Mat &Q, cv::cuda::Stream &stream) {
	// Q(3, 2) = -1/Tx
	// Q(2, 3) = f

	double val = (1.0f / Q.at<double>(3, 2)) * Q.at<double>(2, 3);
	cv::cuda::divide(val, disparity, depth, 1.0f / 1000.0f, -1, stream);
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
	pipeline_->apply(frame, frame, (ftl::rgbd::Source*) lsrc_, cv::cuda::StreamAccessor::getStream(stream2_));

#ifdef HAVE_OPTFLOW
	// see comments in https://gitlab.utu.fi/nicolas.pope/ftl/issues/155
	/*
	if (use_optflow_)
	{
		auto &left_gray = frame.create<cv::cuda::GpuMat>(Channel::LeftGray);
		auto &right_gray = frame.create<cv::cuda::GpuMat>(Channel::RightGray);

		cv::cuda::cvtColor(left, left_gray, cv::COLOR_BGR2GRAY, 0, stream2_);
		cv::cuda::cvtColor(right, right_gray, cv::COLOR_BGR2GRAY, 0, stream2_);

		if (frames_[1].hasChannel(Channel::LeftGray))
		{
			//frames_[1].download(Channel::LeftGray);
			auto &left_gray_prev = frames_[1].get<cv::cuda::GpuMat>(Channel::LeftGray);
			auto &optflow = frame.create<cv::cuda::GpuMat>(Channel::Flow);
			nvof_->calc(left_gray, left_gray_prev, optflow, stream2_);
			// nvof_->upSampler() isn't implemented with CUDA
			// cv::cuda::resize() does not work wiht 2-channel input
			// cv::cuda::resize(optflow_, optflow, left.size(), 0.0, 0.0, cv::INTER_NEAREST, stream2_);
		}
	}*/
#endif

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
	auto &left = frame.get<cv::cuda::GpuMat>(Channel::Left);
	auto &right = frame.get<cv::cuda::GpuMat>(Channel::Right);

	const ftl::codecs::Channel chan = host_->getChannel();
	if (left.empty() || right.empty()) return false;

	if (chan == Channel::Depth) {
		disp_->compute(frame, stream_);
		
		auto &disp = frame.get<cv::cuda::GpuMat>(Channel::Disparity);
		auto &depth = frame.create<cv::cuda::GpuMat>(Channel::Depth);
		if (depth.empty()) depth = cv::cuda::GpuMat(left.size(), CV_32FC1);

		ftl::cuda::disparity_to_depth(disp, depth, params_, stream_);
		
		//left.download(rgb_, stream_);
		//depth.download(depth_, stream_);
		//frame.download(Channel::Left + Channel::Depth);
		stream_.waitForCompletion();
		host_->notify(timestamp_, left, depth);
	} else if (chan == Channel::Right) {
		//left.download(rgb_, stream_);
		//right.download(depth_, stream_);
		stream_.waitForCompletion();  // TODO:(Nick) Move to getFrames
		host_->notify(timestamp_, left, right);
	} else {
		//left.download(rgb_, stream_);
		stream_.waitForCompletion();  // TODO:(Nick) Move to getFrames
		//LOG(INFO) << "NO SECOND CHANNEL: " << (bool)depth_.empty();
		depth_.create(left.size(), left.type());
		host_->notify(timestamp_, left, depth_);
	}

	return true;
}

bool StereoVideoSource::isReady() {
	return ready_;
}
