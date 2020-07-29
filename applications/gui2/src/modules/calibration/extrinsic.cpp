
#include "calibration.hpp"
#include "../../screen.hpp"
#include "../../widgets/popupbutton.hpp"
#include "../../views/calibration/extrinsicview.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/cudawarping.hpp>

#include <ftl/calibration/optimize.hpp>
#include <ftl/calibration/structures.hpp>
#include <ftl/threads.hpp>

#include <nanogui/entypo.h>

using ftl::gui2::Calibration;

using ftl::calibration::CalibrationData;
using ftl::codecs::Channel;
using ftl::data::FrameID;
using ftl::data::FrameSetPtr;

using ftl::gui2::ExtrinsicCalibration;
using ftl::calibration::CalibrationObject;
using ftl::calibration::ArUCoObject;

using ftl::calibration::transform::inverse;
using ftl::calibration::transform::getRotationAndTranslation;

void ExtrinsicCalibration::init() {
	reset();
}

void ExtrinsicCalibration::reset() {
	if(future_.valid()) { future_.wait(); }
	state_ = ExtrinsicCalibration::State();
	running_ = false;
	fs_current_.reset();
	fs_update_.reset();

	state_.calib_object = std::unique_ptr<CalibrationObject>(new ArUCoObject(cv::aruco::DICT_6X6_100));
	state_.calib.points().setObject(state_.calib_object->object());
	state_.min_cameras = 2;
}

ExtrinsicCalibration::~ExtrinsicCalibration() {
	if(future_.valid()) {
		future_.wait();
	}
}

void ExtrinsicCalibration::start(unsigned int fsid, std::vector<FrameID> sources) {

	setCalibrationMode(false);
	reset();

	state_.cameras.reserve(sources.size()*2);

	auto* filter = io->feed()->filter
		(std::unordered_set<uint32_t>{fsid}, {Channel::Left, Channel::Right});

	filter->on([this](const FrameSetPtr& fs){ return onFrameSet_(fs);});

	while(fs_current_ == nullptr) {
		auto fss = filter->getLatestFrameSets();
		if (fss.size() == 1) { fs_current_ = fss.front(); }
	}

	for (auto id : sources) {
		// stereo calibration
		auto cl = CameraID(id.frameset(), id.source(), Channel::Left);
		auto cr = CameraID(id.frameset(), id.source(), Channel::Right);
		const auto& frame = (*fs_current_)[id.source()].cast<ftl::rgbd::Frame>();
		auto sz = cv::Size((int) frame.getLeftCamera().width, (int) frame.getLeftCamera().height);
		state_.cameras.push_back({cl, {}});
		state_.cameras.push_back({cr, {}});

		state_.calib.addStereoCamera(
			CalibrationData::Intrinsic(getCalibration(cl).intrinsic, sz),
			CalibrationData::Intrinsic(getCalibration(cr).intrinsic, sz));
	}

	// initialize last points structure; can't be resized while running (without
	// mutex)
	unsigned int npoints = state_.calib_object->object().size();
	state_.points_prev.resize(state_.cameras.size());
	for (unsigned int i = 0; i < state_.cameras.size(); i++) {
		state_.points_prev[i] = std::vector<cv::Point2d>(npoints);
	}

	auto* view = new ftl::gui2::ExtrinsicCalibrationView(screen, this);
	view->onClose([this, filter](){
		running_ = false;
		filter->remove();
		if (fs_current_ == nullptr) { return; }

		// change mode only once per frame (cameras contain same frame twice)
		std::unordered_set<uint32_t> fids;
		for (const auto camera : state_.cameras) {
			fids.insert(camera.id.source());
		}

		for (const auto i : fids) {
			setCalibrationMode((*fs_current_)[i], true);
		}
	});
	state_.capture = true;
	screen->setView(view);
}

CalibrationData::Calibration ExtrinsicCalibration::calibration(int c) {
	return state_.calib.calibration(c);
}

bool ExtrinsicCalibration::onFrameSet_(const FrameSetPtr& fs) {

	std::atomic_store(&fs_update_, fs);
	screen->redraw();

	bool all_good = true;
	for (const auto& [id, channel] : state_.cameras) {
		std::ignore = channel;
		all_good &= checkFrame((*fs)[id.source()]);
	}
	//if (!all_good) { return true; }

	if (!state_.capture) { return true; }
	if (running_.exchange(true)) { return true; }

	future_ = ftl::pool.push([this, fs = fs](int thread_id) {

		cv::Mat K;
		cv::Mat distCoeffs;
		std::vector<cv::Point2d> points;
		int count = 0;

		for (unsigned int i = 0; i < state_.cameras.size(); i++) {
			const auto& [id, calib] = state_.cameras[i];

			if (!(*fs)[id.source()].hasChannel(id.channel)) { continue; }

			points.clear();
			const cv::cuda::GpuMat& im = (*fs)[id.source()].get<cv::cuda::GpuMat>(id.channel);
			K = calib.intrinsic.matrix();
			distCoeffs = calib.intrinsic.distCoeffs.Mat();

			try {
				int n = state_.calib_object->detect(im, points, K, distCoeffs);
				if (n > 0) {
					state_.calib.points().addPoints(i, points);
					state_.points_prev[i] = points;
					count++;
				}
			}
			catch (std::exception& ex) {
				LOG(ERROR) << ex.what();
			}
		}

		if (count < state_.min_cameras) {
			state_.calib.points().clear();
		}
		else {
			state_.calib.points().next();
		}
		running_ = false;
	});

	return true;
}

bool ExtrinsicCalibration::hasFrame(int camera) {
	const auto id = state_.cameras[camera].id;
	return	(std::atomic_load(&fs_current_).get() != nullptr) &&
			((*fs_current_)[id.source()].hasChannel(id.channel));
}

const cv::cuda::GpuMat ExtrinsicCalibration::getFrame(int camera) {
	const auto id = state_.cameras[camera].id;
	return (*fs_current_)[id.source()].cast<ftl::rgbd::Frame>().get<cv::cuda::GpuMat>(id.channel);
}

const cv::cuda::GpuMat ExtrinsicCalibration::getFrameRectified(int c) {
	if (running_ || state_.maps1.size() <= (unsigned int)(c)) {
		return getFrame(c);
	}
	cv::cuda::GpuMat remapped;
	cv::cuda::remap(getFrame(c), remapped, state_.maps1[c], state_.maps2[c], cv::INTER_LINEAR);
	return remapped;
}

int ExtrinsicCalibration::cameraCount() {
	return state_.cameras.size();
}

bool ExtrinsicCalibration::next() {
	if (std::atomic_load(&fs_update_).get()) {
		std::atomic_store(&fs_current_, fs_update_);
		std::atomic_store(&fs_update_, {});
		return true;
	}
	return false;
}

bool ExtrinsicCalibration::capturing() {
	return state_.capture;
}

void ExtrinsicCalibration::setCapture(bool v) {
	state_.capture = v;
}

std::vector<std::pair<std::string, unsigned int>> ExtrinsicCalibration::listFrameSets() {
	auto framesets = io->feed()->listFrameSets();
	std::vector<std::pair<std::string, unsigned int>> result;
	result.reserve(framesets.size());
	for (auto fsid : framesets) {
		auto uri = io->feed()->getURI(fsid);
		result.push_back({uri, fsid});
	}
	return result;
}

std::vector<std::pair<std::string, ftl::data::FrameID>> ExtrinsicCalibration::listSources(unsigned int fsid, bool all) {
	std::vector<std::pair<std::string, FrameID>> cameras;
	auto fs = io->feed()->getFrameSet(fsid);
	for (auto id : io->feed()->listFrames()) {
		if (id.frameset() != fsid) { continue; }
		if (all || io->feed()->availableChannels(id).count(Channel::CalibrationData)) {
			std::string name = (*fs)[id.source()].name();
			cameras.push_back({name, id});
		}
	}
	return cameras;
}

std::vector<ExtrinsicCalibration::CameraID> ExtrinsicCalibration::cameras() {
	std::vector<ExtrinsicCalibration::CameraID> res;
	res.reserve(cameraCount());
	for (const auto& camera : state_.cameras) {
		res.push_back(camera.id);
	}
	return res;
}

bool ExtrinsicCalibration::isBusy() {
	return running_;
}

void ExtrinsicCalibration::updateCalibration() {
	auto fs = std::atomic_load(&fs_current_);
	std::map<ftl::data::FrameID, ftl::calibration::CalibrationData> update;

	for (unsigned int i = 0; i < state_.cameras.size(); i++) {
		auto& c = state_.cameras[i];
		auto frame_id = ftl::data::FrameID(c.id);

		if (update.count(frame_id) == 0) {
			auto& frame = fs->frames[c.id];
			update[frame_id] = frame.get<CalibrationData>(Channel::CalibrationData);
		}
		update[frame_id].get(c.id.channel) = state_.calib.calibrationOptimized(i);
	}

	for (auto& [fid, calib] : update) {
		auto& frame = fs->frames[fid];
		setCalibration(frame, calib);
	}
}

void ExtrinsicCalibration::updateCalibration(int c) {
	throw ftl::exception("Not implemented");
}

void ExtrinsicCalibration::stereoRectify(int cl, int cr,
	const CalibrationData::Calibration& l, const CalibrationData::Calibration& r) {

	CHECK(l.extrinsic.tvec != r.extrinsic.tvec);
	CHECK(l.intrinsic.resolution == r.intrinsic.resolution);

	auto size = l.intrinsic.resolution;
	cv::Mat T = r.extrinsic.matrix() * inverse(l.extrinsic.matrix());
	cv::Mat R, t, R1, R2, P1, P2, Q, map1, map2;

	getRotationAndTranslation(T, R, t);

	cv::stereoRectify(
		l.intrinsic.matrix(), l.intrinsic.distCoeffs.Mat(),
		r.intrinsic.matrix(), r.intrinsic.distCoeffs.Mat(), size,
		R, t, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 1.0);

	cv::initUndistortRectifyMap(l.intrinsic.matrix(), l.intrinsic.distCoeffs.Mat(),
		R1, P1, size, CV_32FC1, map1, map2);
	state_.maps1[cl].upload(map1);
	state_.maps2[cl].upload(map2);

	cv::initUndistortRectifyMap(r.intrinsic.matrix(), r.intrinsic.distCoeffs.Mat(),
		R2, P2, size, CV_32FC1, map1, map2);
	state_.maps1[cr].upload(map1);
	state_.maps2[cr].upload(map2);
}

void ExtrinsicCalibration::run() {
	if (running_.exchange(true)) { return; }

	future_ = ftl::pool.push([this](int id) {
		try {
			auto opt = state_.calib.options();
			opt.optimize_intrinsic = !(state_.flags & Flags::FIX_INTRINSIC);
			opt.rational_model = state_.flags & Flags::RATIONAL_MODEL;
			opt.fix_focal = state_.flags & Flags::FIX_FOCAL;
			opt.fix_distortion = state_.flags & Flags::FIX_DISTORTION;
			opt.zero_distortion = state_.flags & Flags::ZERO_DISTORTION;
			opt.fix_principal_point = state_.flags & Flags::FIX_PRINCIPAL_POINT;
			opt.loss = (state_.flags & Flags::LOSS_CAUCHY) ?
				ftl::calibration::BundleAdjustment::Options::Loss::CAUCHY :
				ftl::calibration::BundleAdjustment::Options::Loss::SQUARED;
			opt.use_nonmonotonic_steps = state_.flags & Flags::NONMONOTONIC_STEP;

			state_.calib.setOptions(opt);
			state_.calib.run();

			// Rectification maps for visualization; stereo cameras assumed
			// if non-stereo cameras added visualization/grouping (by index)
			// has to be different.

			state_.maps1.resize(cameraCount());
			state_.maps2.resize(cameraCount());

			for (int c = 0; c < cameraCount(); c += 2) {
				auto l = state_.calib.calibrationOptimized(c);
				auto r = state_.calib.calibrationOptimized(c + 1);
				stereoRectify(c, c + 1, l, r);

				LOG(INFO) << "baseline (" << c << ", " << c + 1 << "): "
						  << cv::norm(l.extrinsic.tvec, r.extrinsic.tvec);
			}
		}
		catch (ftl::exception &ex) {
			LOG(ERROR) << ex.what() << "\n" << ex.trace();
		}
		catch (std::exception &ex) {
			LOG(ERROR) << ex.what();
		}

		running_ = false;
	});
}

double ExtrinsicCalibration::reprojectionError(int camera) {
	if (camera <= cameraCount()) {
		return NAN;
	}
	if (camera < 0) {
		return state_.calib.reprojectionError();
	}
	else {
		return state_.calib.reprojectionError(camera);
	}
}

ftl::calibration::CalibrationData::Calibration ExtrinsicCalibration::getCalibration(CameraID id) {
	if (fs_current_ == nullptr) {
		throw ftl::exception("No frame");
	}

	auto calib = (*fs_current_)[id.source()].get<CalibrationData>(Channel::CalibrationData);
	if (!calib.hasCalibration(id.channel)) {
		throw ftl::exception("Calibration missing for requierd channel");
	}

	return calib.get(id.channel);
}

const std::vector<cv::Point2d>& ExtrinsicCalibration::previousPoints(int camera) {
	// not really thread safe (but points_prev_ should not resize)
	return state_.points_prev[camera];
}

int ExtrinsicCalibration::getFrameCount(int camera) {
	return state_.calib.points().getCount(unsigned(camera));
}

void ExtrinsicCalibration::setFlags(int flags) {
	state_.flags = flags;
}

int ExtrinsicCalibration::flags() const {
	return state_.flags;
}

// debug method: save state to file (msgpack)
void ExtrinsicCalibration::saveInput(const std::string& filename) {
	ftl::pool.push([this, filename](int){
		do {
			// calib must not be modified; would be better to have mutex here
			state_.capture = false;
		}
		while(running_);

		running_ = true;
		try { state_.calib.toFile(filename);}
		catch (std::exception& ex) { LOG(ERROR) << "Calib save failed " << ex.what(); }
		running_ = false;
	});
}

// debug method: load state from file (msgpack)
void ExtrinsicCalibration::loadInput(const std::string& filename) {	ftl::pool.push([this, filename](int){
		do {
			// calib must not be modified; would be better to have mutex here
			state_.capture = false;
		}
		while(running_);

		running_ = true;
		try { state_.calib.fromFile(filename); }
		catch (std::exception& ex) { LOG(ERROR) << "Calib load failed: " << ex.what(); }
		running_ = false;
	});
}
