#include <loguru.hpp>

#include "calibration.hpp"
#include "../../screen.hpp"
#include "../../widgets/popupbutton.hpp"
#include "../../views/calibration/intrinsicview.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <ftl/calibration/structures.hpp>
#include <ftl/threads.hpp>

#include <nanogui/entypo.h>

using ftl::gui2::Calibration;
using ftl::gui2::IntrinsicCalibration;

using ftl::calibration::ChessboardObject;
using ftl::calibration::CalibrationData;
using ftl::codecs::Channel;
using ftl::data::FrameID;
using ftl::data::FrameSetPtr;

void IntrinsicCalibration::init() {
	reset();
}

IntrinsicCalibration::~IntrinsicCalibration() {
	if(future_.valid()) {
		future_.wait();
	}
}

cv::Size IntrinsicCalibration::chessboardSize() {
	return state_->object->chessboardSize();
}

double IntrinsicCalibration::squareSize() {
	return state_->object->squareSize();
}

void IntrinsicCalibration::setChessboard(cv::Size size, double square) {
	state_->object = std::make_unique<ChessboardObject>(size.height, size.width, square);
}

void IntrinsicCalibration::reset() {
	state_ = std::make_unique<State>();
	state_->object = std::make_unique<ChessboardObject>();
	state_->channel = Channel::Left;
	state_->channel_alt = Channel::Left;
	state_->flags.set(defaultFlags());
}

void IntrinsicCalibration::start(ftl::data::FrameID id) {
	reset();
	setCalibrationMode(false);

	state_->id = id;

	auto* filter = io->feed()->filter
		(std::unordered_set<uint32_t>{id.frameset()},
		 {Channel::Left, Channel::Right});

	filter->on([this](const FrameSetPtr& fs){ return onFrame_(fs); });

	while(fs_current_ == nullptr) {
		auto fss = filter->getLatestFrameSets();
		if (fss.size() == 1) { fs_current_ = fss.front(); }
	}
	auto fs = std::atomic_load(&fs_current_);
	setChannel_(fs);

	auto* view = new ftl::gui2::IntrinsicCalibrationView(screen, this);
	view->onClose([filter, this](){
		// if calib_ caches images, also reset() here!
		filter->remove();
		if (fs_current_) {
			setCalibrationMode(fs_current_->frames[state_->id.source()], true);
		}
		reset();
		fs_current_.reset();
		fs_update_.reset();
	});

	screen->setView(view);
}

void IntrinsicCalibration::setChannel(Channel channel) {
	state_->channel = channel;
	auto fs = std::atomic_load(&fs_current_);
	setChannel_(fs);
}

void IntrinsicCalibration::setChannel_(FrameSetPtr fs) {
	// reset points, find if high res available and find correct resolution
	// TODO/FIXME: channel might be missing from previous frameset; temporary
	// fix uses left channel to set resulution (assumes left resolution always
	// the same as right resolution).

	state_->calib = CalibrationData::Intrinsic();
	state_->points.clear();
	state_->points_object.clear();
	state_->count = 0;

	state_->channel_alt = state_->channel;
	if (fs == nullptr) {
		LOG(ERROR) << "No frame, calibration not loaded";
	}

	auto& frame = (*fs)[state_->id.source()].cast<ftl::rgbd::Frame>();
	cv::Size size;

	if (state_->channel== Channel::Left) {
		if(frame.has(Channel::LeftHighRes)) {
			state_->channel_alt = Channel::LeftHighRes;
			size = frame.get<cv::Mat>(state_->channel_alt).size();
		}
		else {
			size = frame.get<cv::Mat>(state_->channel_alt).size();
		}
	}
	else if (state_->channel== Channel::Right) {
		if (frame.has(Channel::RightHighRes)) {
			state_->channel_alt = Channel::RightHighRes;
			size = frame.get<cv::Mat>(Channel::LeftHighRes).size();
		}
		else {
			size = frame.get<cv::Mat>(Channel::Left).size();
		}
	}

	try {
		auto calib = frame.get<CalibrationData>(Channel::CalibrationData);
		if (calib.hasCalibration(state_->channel)) {
			auto intrinsic = calib.get(state_->channel).intrinsic;
			state_->calib = CalibrationData::Intrinsic(intrinsic, size);
			state_->calibrated = true;
		}
		else {
			state_->calib.resolution = size;
		}
	}
	catch (std::exception& ex) {
		LOG(ERROR)	<< "Could not read calibration: " << ex.what()
					<< "; is this a valid source?";
	}
}

bool IntrinsicCalibration::onFrame_(const ftl::data::FrameSetPtr& fs) {

	std::atomic_store(&fs_update_, fs);
	screen->redraw();

	auto& frame = fs->frames[state_->id.source()];

	if (!checkFrame(frame)) { return true; }
	if (!state_->capture) { return true; }
	if ((float(glfwGetTime()) - state_->last) < state_->frequency) { return true; }
	if (state_->running.exchange(true)) { return true; }

	future_ = ftl::pool.push(	[fs, this]
								(int thread_id) {

		try {
			auto& frame = (*fs)[state_->id.source()].cast<ftl::rgbd::Frame>();

			auto im = getMat(frame, state_->channel_alt);
			cv::cvtColor(im, state_->gray, cv::COLOR_BGRA2GRAY);

			std::vector<cv::Point2d> points;
			int npoints = state_->object->detect(state_->gray, points);

			if (npoints > 0) {
				std::unique_lock<std::mutex> lk(mtx_);

				auto& new_points = state_->points.emplace_back();
				for (auto p : points) {
					new_points.push_back(p);
				}

				auto& new_points_obj = state_->points_object.emplace_back();
				for (auto p : state_->object->object()) {
					new_points_obj.push_back(p);
				}

				state_->count++;
			}
			else {
				LOG(INFO) << "Calibration pattern was not detected";
			}
		}
		catch (std::exception &e) {
			LOG(ERROR) << "exception in chesboard detection: " << e.what();
			state_->running = false;
			throw;
		}

		state_->running = false;
		state_->last = float(glfwGetTime());
	});

	return true;
}


void IntrinsicCalibration::save() {
	auto& frame = fs_current_->frames[state_->id.source()];
	CalibrationData calib_data = CalibrationData(frame.get<CalibrationData>(Channel::CalibrationData));
	auto& calibration = calib_data.get(state_->channel);
	calibration.intrinsic = state_->calib;
	setCalibration(frame, calib_data);
}

int IntrinsicCalibration::defaultFlags() {
	int flags = state_->flags.defaultFlags();

	// load config flags
	for (int i : state_->flags.list()) {
		auto flag = get<bool>(state_->flags.name(i));
		if (flag) {
			if (*flag)	flags |= i;
			else		flags &= (~i);
		}
	}

	return flags;
}

bool IntrinsicCalibration::isBusy() {
	return state_->capture || state_->running;
}

void IntrinsicCalibration::run() {
	state_->running = true;
	future_ = ftl::pool.push([this](int id) {
		try {
			for (auto f : state_->flags.list()) {
				if (state_->flags.has(f)) {
					LOG(INFO) << state_->flags.name(f);
				}
			}
			cv::Size2d ssize = sensorSize();
			cv::Mat K;
			cv::Mat distCoeffs;
			cv::Size size = state_->calib.resolution;
			if (state_->flags.has(cv::CALIB_USE_INTRINSIC_GUESS)) {
				// OpenCV seems to use these anyways?
				K = state_->calib.matrix();
				state_->calib.distCoeffs.Mat(12).copyTo(distCoeffs);
			}
			std::vector<cv::Mat> rvecs, tvecs;
			auto term = cv::TermCriteria
				(cv::TermCriteria::COUNT|cv::TermCriteria::EPS, state_->max_iter, 1.0e-6);

			state_->reprojection_error = cv::calibrateCamera(
				state_->points_object, state_->points,
				size, K, distCoeffs, rvecs, tvecs,
				state_->flags, term);

			state_->calib = CalibrationData::Intrinsic(K, distCoeffs, size);
			state_->calib.sensorSize = ssize;
			state_->calibrated = true;
		}
		catch (std::exception &e) {
			LOG(ERROR) << "exception in calibration: " << e.what();
			state_->running = false;
			throw;
		}

		state_->running = false;
	});
}

bool IntrinsicCalibration::hasFrame() {
	return (std::atomic_load(&fs_update_).get() != nullptr)
		&& fs_update_->frames[state_->id.source()].hasChannel(state_->channel_alt);
};

cv::cuda::GpuMat IntrinsicCalibration::getFrame() {
	if (std::atomic_load(&fs_update_)) {
		fs_current_ = fs_update_;
		std::atomic_store(&fs_update_, {});
	}

	if (!fs_current_) {
		return cv::cuda::GpuMat();
	}

	return getGpuMat((*fs_current_)[state_->id.source()].cast<ftl::rgbd::Frame>(),
					 state_->channel_alt);
}

cv::cuda::GpuMat IntrinsicCalibration::getFrameUndistort() {
	if (!calibrated()) {
		return getFrame();
	}

	if (std::atomic_load(&fs_update_)) {
		fs_current_ = fs_update_;
		std::atomic_store(&fs_update_, {});
	}

	if (!fs_current_) {
		return cv::cuda::GpuMat();
	}

	auto im = getMat((*fs_current_)[state_->id.source()].cast<ftl::rgbd::Frame>(),
					 state_->channel_alt);

	// NOTE: would be faster to use remap() and computing the maps just once if
	// performance is relevant here

	cv::Mat im_undistort;
	cv::cuda::GpuMat gpu;
	cv::undistort(im, im_undistort, state_->calib.matrix(), state_->calib.distCoeffs.Mat(12));
	gpu.upload(im_undistort);
	return gpu;
}

cv::Size2d IntrinsicCalibration::sensorSize() {
	if (state_->calib.sensorSize == cv::Size2d{0.0, 0.0}) {
		double w = value("sensor_width", 0.0);
		double h = value("sensor_height", 0.0);
		return {w, h};
	}
	else {
		return state_->calib.sensorSize;
	}
};

void IntrinsicCalibration::setSensorSize(cv::Size2d sz) {
	state_->calib.sensorSize = sz;
}

double IntrinsicCalibration::focalLength() {
	return (state_->calib.fx)*(sensorSize().width/state_->calib.resolution.width);
}

void IntrinsicCalibration::setFocalLength(double value, cv::Size2d sensor_size) {
	setSensorSize(sensor_size);
	double f = value*(state_->calib.resolution.width/sensor_size.width);

	state_->calib.cx = f;
	state_->calib.cy = f;
}

void IntrinsicCalibration::resetPrincipalPoint() {
	auto sz = state_->calib.resolution;
	state_->calib.cx = double(sz.width)/2.0;
	state_->calib.cy = double(sz.height)/2.0;
}

void IntrinsicCalibration::resetDistortion() {
	state_->calib.distCoeffs = CalibrationData::Intrinsic::DistortionCoefficients();
}


bool IntrinsicCalibration::hasChannel(Channel c) {
	if (fs_current_) {
		return (*fs_current_)[state_->id.source()].hasChannel(c);
	}
	return false;
}

std::vector<std::pair<std::string, FrameID>> IntrinsicCalibration::listSources(bool all) {
	std::vector<std::pair<std::string, FrameID>> cameras;
	for (auto id : io->feed()->listFrames()) {
		auto channels = io->feed()->availableChannels(id);
		if (all || (channels.count(Channel::CalibrationData) == 1)) {
			auto name = (*(io->feed()->getFrameSet(id.frameset())))[id.source()].name();
			//auto name = io->feed()->getURI(id.frameset()) + "#" + std::to_string(id.source());
			cameras.push_back({name, id});
		}
	}
	return cameras;
}

std::vector<cv::Point2f> IntrinsicCalibration::previousPoints() {
	std::unique_lock<std::mutex> lk(mtx_, std::defer_lock);
	if (lk.try_lock()) {
		if (state_->points.size() == 0) { return {}; }
		return std::vector<cv::Point2f>(state_->points.back());
	}
	return {};
}

ftl::calibration::CalibrationData::Intrinsic IntrinsicCalibration::calibration() {
	return state_->calib;
}

