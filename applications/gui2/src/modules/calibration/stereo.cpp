#include <loguru.hpp>

#include "calibration.hpp"
#include "../../screen.hpp"
#include "../../widgets/popupbutton.hpp"
#include "../../views/calibration/stereoview.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <ftl/calibration/parameters.hpp>
#include <ftl/calibration/structures.hpp>
#include <ftl/threads.hpp>

#include <nanogui/entypo.h>

using ftl::gui2::Calibration;
using ftl::gui2::StereoCalibration;

using ftl::calibration::ChessboardObject;
using ftl::calibration::CalibrationData;
using ftl::codecs::Channel;
using ftl::data::FrameID;
using ftl::data::FrameSetPtr;

////////////////////////////////////////////////////////////////////////////////

void StereoCalibration::setCapture(bool v) {
	state_->capture = v;
}

bool StereoCalibration::capturing() {
	return state_->capture;
}

void StereoCalibration::setFrequency(float v) {
	state_->frequency = v;
}

float StereoCalibration::frequency() {
	return state_->frequency;
}

void StereoCalibration::init() {
	state_ = std::make_unique<State>();
	state_->object = std::unique_ptr<ChessboardObject>(new ChessboardObject());
}

StereoCalibration::~StereoCalibration() {
	if (state_) {
		state_->running = false;
	}
	if(future_.valid()) {
		future_.wait();
	}
	fs_current_.reset();
	fs_update_.reset();
}

void StereoCalibration::reset() {
	while(state_->running) { state_->capture = false; }
	state_ = std::make_unique<State>();
	state_->object = std::unique_ptr<ChessboardObject>(new ChessboardObject());
	resetFlags();
}

cv::Size StereoCalibration::chessboardSize() {
	return state_->object->chessboardSize();
}

double StereoCalibration::squareSize() {
	return state_->object->squareSize();
}

void StereoCalibration::setChessboard(cv::Size size, double square) {
	state_->object = std::make_unique<ChessboardObject>(size.height, size.width, square);
}

void StereoCalibration::start(ftl::data::FrameID id) {
	reset();
	setCalibrationMode(false);
	state_->id = id;

	auto* view = new ftl::gui2::StereoCalibrationView(screen, this);
	auto* filter = io->feed()->filter
		(std::unordered_set<uint32_t>{id.frameset()},
		 {Channel::Left, Channel::Right});

	filter->on([this](const FrameSetPtr& fs){ return onFrame_(fs); });

	view->onClose([filter, this](){
		// if state_->calib caches images, also reset() here!
		filter->remove();
		if (fs_current_) {
			setCalibrationMode(fs_current_->frames[state_->id.source()], true);
		}
		reset();
		fs_current_.reset();
		fs_update_.reset();
	});

	screen->setView(view);

	for (auto fs : filter->getLatestFrameSets()) {
		if (!(fs->frameset() == state_->id.frameset()) ||
			!(fs->hasFrame(state_->id.source()))) { continue; }

		// read calibration channel and set channel_alt_ to high res if available

		try {
			auto& frame = (*fs)[state_->id.source()];
			state_->calib = frame.get<CalibrationData>(Channel::CalibrationData);
			state_->highres = false; // TODO: Remove
			auto sizel = frame.get<cv::cuda::GpuMat>(channelLeft_()).size();
			auto sizer = frame.get<cv::cuda::GpuMat>(channelLeft_()).size();
			if (sizel != sizer) {
				LOG(ERROR) << "Frames have different resolutions";
				// TODO: do not proceed
			}
			state_->imsize = sizel;
		}
		catch (std::exception& ex) {
			LOG(ERROR)	<< "Could not read calibration: " << ex.what()
						<< "; is this a valid source?";
		}
		break;
	}
}

bool StereoCalibration::onFrame_(const ftl::data::FrameSetPtr& fs) {

	std::atomic_store(&fs_update_, fs);
	screen->redraw();

	auto& frame = fs->frames[state_->id.source()];

	if (!checkFrame(frame)) { return true; }
	if (!frame.hasAll({channelLeft_(), channelRight_()})) { return true; }
	if (!state_->capture) { return true; }
	if ((float(glfwGetTime()) - state_->last) < state_->frequency) { return true; }
	if (state_->running.exchange(true)) { return true; }

	future_ = ftl::pool.push([this, fs] (int thread_id) {

		try {
			auto& frame = (*fs)[state_->id.source()].cast<ftl::rgbd::Frame>();
			auto l = getMat(frame, channelLeft_());
			auto r = getMat(frame, channelRight_());
			cv::cvtColor(l, state_->gray_left, cv::COLOR_BGRA2GRAY);
			cv::cvtColor(r, state_->gray_right, cv::COLOR_BGRA2GRAY);

			std::vector<cv::Point2d> pointsl;
			std::vector<cv::Point2d> pointsr;
			if ((state_->object->detect(state_->gray_left, pointsl) == 1) &&
				(state_->object->detect(state_->gray_right, pointsr) == 1)) {

				std::unique_lock<std::mutex> lk(mtx_);
				auto& new_points_l = state_->points_l.emplace_back();
				new_points_l.reserve(pointsl.size());
				auto& new_points_r = state_->points_r.emplace_back();
				new_points_r.reserve(pointsl.size());
				auto& new_points_obj = state_->points_object.emplace_back();
				new_points_obj.reserve(pointsl.size());

				for (auto p : pointsl) { new_points_l.push_back(p); }
				for (auto p : pointsr) { new_points_r.push_back(p); }
				for (auto p : state_->object->object()) { new_points_obj.push_back(p); }
				state_->count++;
			}
		}
		catch (std::exception &e) {
			LOG(ERROR) << "exception in chesboard detection: " << e.what();
			running = false;
			throw;
		}

		state_->running = false;
		state_->last = float(glfwGetTime());
	});

	return true;
}


void StereoCalibration::saveCalibration() {
	auto fs = std::atomic_load(&(fs_current_));
	setCalibration((*fs)[state_->id.source()], state_->calib);
}

void StereoCalibration::resetFlags() {
	// reset flags and get class defaults
	state_->flags.reset();
	state_->flags.set(state_->flags.defaultFlags());

	// load config flags
	for (int i : state_->flags.list()) {
		auto flag = get<bool>(state_->flags.name(i));
		if (flag) {
			if (*flag)	state_->flags.set(i);
			else		state_->flags.unset(i);
		}
	}
}

bool StereoCalibration::isBusy() {
	return state_->capture || state_->running;
}

void StereoCalibration::run() {
	if (state_->running) { return; }

	state_->running = true;
	future_ = ftl::pool.push([this](int) {
		try {
			auto& calib_l = state_->calib.get(Channel::Left);
			auto& calib_r = state_->calib.get(Channel::Right);
			auto K1 = calib_l.intrinsic.matrix();
			auto distCoeffs1 = calib_l.intrinsic.distCoeffs.Mat();
			auto K2 = calib_l.intrinsic.matrix();
			auto distCoeffs2 = calib_r.intrinsic.distCoeffs.Mat();
			cv::Mat R, T, E, F;
			state_->reprojection_error = cv::stereoCalibrate(
				state_->points_object, state_->points_l,
				state_->points_r, K1, distCoeffs1, K2, distCoeffs2,
				state_->imsize, R, T, E, F, state_->flags);

			state_->calib.get(Channel::Left).intrinsic =
				CalibrationData::Intrinsic(K1, distCoeffs1, state_->imsize);
			state_->calib.get(Channel::Right).intrinsic =
				CalibrationData::Intrinsic(K2, distCoeffs2, state_->imsize);

			state_->calib.get(Channel::Left).extrinsic = CalibrationData::Extrinsic();
			state_->calib.get(Channel::Right).extrinsic = CalibrationData::Extrinsic(R, T);
		}
		catch (std::exception &e) {
			LOG(ERROR) << "exception in calibration: " << e.what();
			state_->running = false;
			throw;
		}

		state_->running = false;
	});
}

ftl::rgbd::Frame& StereoCalibration::frame_() {
	if (std::atomic_load(&fs_update_)) {
		fs_current_ = fs_update_;
		std::atomic_store(&fs_update_, {});
	}
	return (*fs_current_)[state_->id.source()].cast<ftl::rgbd::Frame>();
}

bool StereoCalibration::hasFrame() {
	auto cleft = Channel::Left;
	auto cright = Channel::Right;
	return (std::atomic_load(&fs_update_).get() != nullptr)
		&& fs_update_->frames[state_->id.source()].hasAll({cleft, cright});
};

Channel StereoCalibration::channelLeft_() {
	return Channel::Left;
}

Channel StereoCalibration::channelRight_() {
	return Channel::Right;
}

cv::cuda::GpuMat StereoCalibration::getLeft() {
	return getGpuMat(frame_() ,channelLeft_());
}

cv::cuda::GpuMat StereoCalibration::getRight() {
	return getGpuMat(frame_() ,channelRight_());
}

bool StereoCalibration::hasChannel(Channel c) {
	if (fs_current_) {
		return (*fs_current_)[state_->id.source()].hasChannel(c);
	}
	return false;
}

std::vector<std::pair<std::string, FrameID>> StereoCalibration::listSources(bool all) {
	std::vector<std::pair<std::string, FrameID>> cameras;
	for (auto id : io->feed()->listFrames()) {
		auto channels = io->feed()->availableChannels(id);
		// TODO: doesn't work
		if (all || (channels.count(Channel::CalibrationData) == 1)) {
			auto name = io->feed()->getURI(id.frameset()) + "#" + std::to_string(id.source());
			cameras.push_back({name, id});
		}
	}
	return cameras;
}

std::vector<std::vector<cv::Point2f>> StereoCalibration::previousPoints() {
	std::unique_lock<std::mutex> lk(mtx_, std::defer_lock);
	if (lk.try_lock()) {
		if (state_->points_l.size() > 0) {
			return {	state_->points_l.back(),
						state_->points_r.back()
			};
		}
	}
	return {};
}

ftl::calibration::CalibrationData::Calibration StereoCalibration::calibrationLeft() {
	return state_->calib.get(Channel::Left);
}

ftl::calibration::CalibrationData::Calibration StereoCalibration::calibrationRight() {
	return state_->calib.get(Channel::Right);
}

bool StereoCalibration::calibrated() {
	return (cv::norm(calibrationLeft().extrinsic.tvec,
					 calibrationRight().extrinsic.tvec) > 0);
}

void StereoCalibration::calculateRectification() {

	using ftl::calibration::transform::inverse;

	auto left = calibrationLeft();
	auto right = calibrationRight();
	auto size = left.intrinsic.resolution;

	cv::Mat T = inverse(left.extrinsic.matrix()) * right.extrinsic.matrix();
	cv::Mat Rl, Rr, Pl, Pr, Q;

	cv::stereoRectify(left.intrinsic.matrix(), left.intrinsic.distCoeffs.Mat(),
					  right.intrinsic.matrix(), right.intrinsic.distCoeffs.Mat(),
					  size, T(cv::Rect(0, 0, 3, 3)), T(cv::Rect(3, 0, 1, 3)),
					  Rl, Rr, Pl, Pr, Q, 0, 1.0, {0, 0},
					  &(state_->validROI_l), &(state_->validROI_r));

	cv::initUndistortRectifyMap(left.intrinsic.matrix(), left.intrinsic.distCoeffs.Mat(),
								Rl, Pl, size, CV_16SC1,
								state_->map_l.first, state_->map_l.second);

	cv::initUndistortRectifyMap(right.intrinsic.matrix(), right.intrinsic.distCoeffs.Mat(),
								Rr, Pr, size, CV_16SC1,
								state_->map_r.first, state_->map_r.second);
}

cv::cuda::GpuMat StereoCalibration::getLeftRectify() {
	if (state_->map_l.first.empty()) { calculateRectification(); }
	cv::Mat tmp;
	cv::cuda::GpuMat res;
	cv::remap(getMat(frame_(), channelLeft_()), tmp,
			  state_->map_l.first,  state_->map_l.second,
			  cv::INTER_LINEAR);
	res.upload(tmp);
	return res;
}
cv::cuda::GpuMat StereoCalibration::getRightRectify() {
	if (state_->map_r.first.empty()) { calculateRectification(); }
	cv::Mat tmp;
	cv::cuda::GpuMat res;
	cv::remap(getMat(frame_(), channelRight_()), tmp,
			  state_->map_r.first,  state_->map_r.second,
			  cv::INTER_LINEAR);
	res.upload(tmp);
	return res;
}
