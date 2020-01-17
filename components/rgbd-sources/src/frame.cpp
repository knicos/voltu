
#include <ftl/rgbd/frame.hpp>

using ftl::rgbd::Frame;
using ftl::rgbd::FrameState;
using ftl::codecs::Channels;
using ftl::codecs::Channel;

static cv::Mat none;
static cv::cuda::GpuMat noneGPU;

FrameState::FrameState() : camera_left_({0}), camera_right_({0}), config_(nlohmann::json::value_t::object) {
	pose_ = Eigen::Matrix4d::Identity();
}

FrameState::FrameState(FrameState &f) {
	pose_ = f.pose_;
	camera_left_ = f.camera_left_;
	camera_right_ = f.camera_right_;
	changed_ = f.changed_;
	config_ = f.config_;
	// TODO: Add mutex lock
	f.changed_.clear();
}

FrameState::FrameState(FrameState &&f) {
	pose_ = f.pose_;
	camera_left_ = f.camera_left_;
	camera_right_ = f.camera_right_;
	changed_ = f.changed_;
	config_ = std::move(f.config_);
	// TODO: Add mutex lock
	f.changed_.clear();
}

FrameState &FrameState::operator=(FrameState &f) {
	pose_ = f.pose_;
	camera_left_ = f.camera_left_;
	camera_right_ = f.camera_right_;
	changed_ = f.changed_;
	config_ = f.config_;
	// TODO: Add mutex lock
	f.changed_.clear();
	return *this;
}

FrameState &FrameState::operator=(FrameState &&f) {
	pose_ = f.pose_;
	camera_left_ = f.camera_left_;
	camera_right_ = f.camera_right_;
	changed_ = f.changed_;
	config_ = std::move(f.config_);
	// TODO: Add mutex lock
	f.changed_.clear();
	return *this;
}

void FrameState::setPose(const Eigen::Matrix4d &pose) {
	pose_ = pose;
	changed_ += Channel::Pose;
}

void FrameState::setLeft(const ftl::rgbd::Camera &p) {
	camera_left_ = p;
	changed_ += Channel::Calibration;
}

void FrameState::setRight(const ftl::rgbd::Camera &p) {
	camera_right_ = p;
	changed_ += Channel::Calibration2;
}

// =============================================================================

void Frame::reset() {
	origin_ = nullptr;
	channels_.clear();
	gpu_.clear();
	for (size_t i=0u; i<Channels<0>::kMax; ++i) {
		data_[i].encoded.clear();
	}
}

void Frame::resetFull() {
	origin_ = nullptr;
	channels_.clear();
	gpu_.clear();
	for (size_t i=0u; i<Channels<0>::kMax; ++i) {
		data_[i].gpu = cv::cuda::GpuMat();
		data_[i].host = cv::Mat();
		data_[i].encoded.clear();
	}
}

void Frame::download(Channel c, cv::cuda::Stream stream) {
	download(Channels(c), stream);
}

void Frame::upload(Channel c, cv::cuda::Stream stream) {
	upload(Channels(c), stream);
}

void Frame::download(Channels<0> c, cv::cuda::Stream stream) {
	for (size_t i=0u; i<Channels<0>::kMax; ++i) {
		if (c.has(i) && channels_.has(i) && gpu_.has(i)) {
			data_[i].gpu.download(data_[i].host, stream);
			gpu_ -= i;
		}
	}
}

void Frame::upload(Channels<0> c, cv::cuda::Stream stream) {
	for (size_t i=0u; i<Channels<0>::kMax; ++i) {
		if (c.has(i) && channels_.has(i) && !gpu_.has(i)) {
			data_[i].gpu.upload(data_[i].host, stream);
			gpu_ += i;
		}
	}
}

void Frame::pushPacket(ftl::codecs::Channel c, ftl::codecs::Packet &pkt) {
	if (hasChannel(c)) {
		auto &m1 = _get(c);
		m1.encoded.emplace_back() = std::move(pkt);
	} else {
		LOG(ERROR) << "Channel " << (int)c << " doesn't exist for packet push";
	}
}

const std::list<ftl::codecs::Packet> &Frame::getPackets(ftl::codecs::Channel c) const {
	if (!hasChannel(c)) {
		throw ftl::exception(ftl::Formatter() << "Frame channel does not exist: " << (int)c);
	}

	auto &m1 = _get(c);
	return m1.encoded;
}

void Frame::mergeEncoding(ftl::rgbd::Frame &f) {
	//LOG(INFO) << "MERGE " << (unsigned int)f.channels_;
	for (auto c : channels_) {
		//if (!f.hasChannel(c)) f.create<cv::cuda::GpuMat>(c);
		if (f.hasChannel(c)) {
			auto &m1 = _get(c);
			auto &m2 = f._get(c);
			m1.encoded.splice(m1.encoded.begin(), m2.encoded);
			//LOG(INFO) << "SPLICED: " << m1.encoded.size();
		}
	}
}

bool Frame::empty(ftl::codecs::Channels<0> channels) {
	for (auto c : channels) {
		if (empty(c)) return true;
	}
	return false;
}

void Frame::swapTo(ftl::codecs::Channels<0> channels, Frame &f) {
	f.reset();
	f.origin_ = origin_;
	f.state_ = state_;

	// For all channels in this frame object
	for (auto c : channels_) {
		// Should we swap this channel?
		if (channels.has(c)) {
			// Does 'f' have this channel?
			//if (!f.hasChannel(c)) {
				// No, so create it first
				// FIXME: Allocate the memory as well?
				if (isCPU(c)) f.create<cv::Mat>(c);
				else f.create<cv::cuda::GpuMat>(c);
			//}

			auto &m1 = _get(c);
			auto &m2 = f._get(c);

			cv::swap(m1.host, m2.host);
			cv::cuda::swap(m1.gpu, m2.gpu);

			auto temptex = std::move(m2.tex);
			m2.tex = std::move(m1.tex);
			m1.tex = std::move(temptex);

			if (m2.encoded.size() > 0 || m1.encoded.size() > 0) {
				auto tempenc = std::move(m2.encoded);
				m2.encoded = std::move(m1.encoded);
				m1.encoded = std::move(tempenc);
			}
		}
	}
}

void Frame::swapChannels(ftl::codecs::Channel a, ftl::codecs::Channel b) {
	auto &m1 = _get(a);
	auto &m2 = _get(b);
	cv::swap(m1.host, m2.host);
	cv::cuda::swap(m1.gpu, m2.gpu);

	auto temptex = std::move(m2.tex);
	m2.tex = std::move(m1.tex);
	m1.tex = std::move(temptex);

	if (m2.encoded.size() > 0 || m1.encoded.size() > 0) {
		auto tempenc = std::move(m2.encoded);
		m2.encoded = std::move(m1.encoded);
		m1.encoded = std::move(tempenc);
	}
}

void Frame::copyTo(ftl::codecs::Channels<0> channels, Frame &f) {
	f.reset();
	f.origin_ = origin_;
	f.state_ = state_;

	// For all channels in this frame object
	for (auto c : channels_) {
		// Should we copy this channel?
		if (channels.has(c)) {
			if (isCPU(c)) get<cv::Mat>(c).copyTo(f.create<cv::Mat>(c));
			else get<cv::cuda::GpuMat>(c).copyTo(f.create<cv::cuda::GpuMat>(c));
			auto &m1 = _get(c);
			auto &m2 = f._get(c);
			m2.encoded = m1.encoded; //std::move(m1.encoded);  // TODO: Copy?
		}
	}
}

template<> cv::Mat& Frame::get(ftl::codecs::Channel channel) {
	if (channel == Channel::None) {
		DLOG(WARNING) << "Cannot get the None channel from a Frame";
		none.release();
		return none;
	}

	if (isGPU(channel)) {
		download(Channels(channel));
		LOG(WARNING) << "Getting GPU channel on CPU without explicit 'download'";
	}

	// Add channel if not already there
	if (!channels_.has(channel)) {
		throw ftl::exception(ftl::Formatter() << "Frame channel does not exist: " << (int)channel);
	}

	return _get(channel).host;
}

template<> cv::cuda::GpuMat& Frame::get(ftl::codecs::Channel channel) {
	if (channel == Channel::None) {
		DLOG(WARNING) << "Cannot get the None channel from a Frame";
		noneGPU.release();
		return noneGPU;
	}

	if (isCPU(channel)) {
		upload(Channels(channel));
		LOG(WARNING) << "Getting CPU channel on GPU without explicit 'upload'";
	}

	// Add channel if not already there
	if (!channels_.has(channel)) {
		throw ftl::exception(ftl::Formatter() << "Frame channel does not exist: " << (int)channel);
	}

	return _get(channel).gpu;
}

template<> const cv::Mat& Frame::get(ftl::codecs::Channel channel) const {
	if (channel == Channel::None) {
		LOG(FATAL) << "Cannot get the None channel from a Frame";
	}

	if (isGPU(channel)) {
		LOG(FATAL) << "Getting GPU channel on CPU without explicit 'download'";
	}

	if (!channels_.has(channel)) throw ftl::exception(ftl::Formatter() << "Frame channel does not exist: " << (int)channel);

	return _get(channel).host;
}

template<> const cv::cuda::GpuMat& Frame::get(ftl::codecs::Channel channel) const {
	if (channel == Channel::None) {
		LOG(FATAL) << "Cannot get the None channel from a Frame";
	}

	if (isCPU(channel)) {
		LOG(FATAL) << "Getting CPU channel on GPU without explicit 'upload'";
	}

	// Add channel if not already there
	if (!channels_.has(channel)) {
		throw ftl::exception(ftl::Formatter() << "Frame channel does not exist: " << (int)channel);
	}

	return _get(channel).gpu;
}

template<> const Eigen::Matrix4d& Frame::get(ftl::codecs::Channel channel) const {
	if (channel == Channel::Pose) {
		return state_.getPose();
	}

	throw ftl::exception(ftl::Formatter() << "Invalid pose channel: " << (int)channel);
}

template<> const ftl::rgbd::Camera& Frame::get(ftl::codecs::Channel channel) const {
	if (channel == Channel::Calibration) {
		return state_.getLeft();
	} else if (channel == Channel::Calibration2) {
		return state_.getRight();
	}

	throw ftl::exception(ftl::Formatter() << "Invalid calibration channel: " << (int)channel);
}

template<> const nlohmann::json& Frame::get(ftl::codecs::Channel channel) const {
	if (channel == Channel::Configuration) {
		return state_.getConfig();
	}

	throw ftl::exception(ftl::Formatter() << "Invalid configuration channel: " << (int)channel);
}

template <> cv::Mat &Frame::create(ftl::codecs::Channel c, const ftl::rgbd::FormatBase &f) {
	if (c == Channel::None) {
		throw ftl::exception("Cannot create a None channel");
	}
	channels_ += c;
	gpu_ -= c;

	auto &m = _get(c);

	m.encoded.clear();  // Remove all old encoded data

	if (!f.empty()) {
		m.host.create(f.size(), f.cvType);
	}

	return m.host;
}

template <> cv::cuda::GpuMat &Frame::create(ftl::codecs::Channel c, const ftl::rgbd::FormatBase &f) {
	if (c == Channel::None) {
		throw ftl::exception("Cannot create a None channel");
	}
	channels_ += c;
	gpu_ += c;

	auto &m = _get(c);

	m.encoded.clear();  // Remove all old encoded data

	if (!f.empty()) {
		m.gpu.create(f.size(), f.cvType);
	}

	return m.gpu;
}

void Frame::clearPackets(ftl::codecs::Channel c) {
	auto &m = _get(c);
	m.encoded.clear();
}

template <> cv::Mat &Frame::create(ftl::codecs::Channel c) {
	if (c == Channel::None) {
		throw ftl::exception("Cannot create a None channel");
	}
	channels_ += c;
	gpu_ -= c;

	auto &m = _get(c);

	m.encoded.clear();  // Remove all old encoded data

	return m.host;
}

template <> cv::cuda::GpuMat &Frame::create(ftl::codecs::Channel c) {
	if (c == Channel::None) {
		throw ftl::exception("Cannot create a None channel");
	}
	channels_ += c;
	gpu_ += c;

	auto &m = _get(c);

	m.encoded.clear();  // Remove all old encoded data

	return m.gpu;
}

void Frame::resetTexture(ftl::codecs::Channel c) {
	auto &m = _get(c);
	m.tex.free();
}

void Frame::setOrigin(ftl::rgbd::FrameState *state) {
	if (origin_ != nullptr) {
		throw ftl::exception("Can only set origin once after reset");
	}

	origin_ = state;
	state_ = *state;
}

const Eigen::Matrix4d &Frame::getPose() const {
	return get<Eigen::Matrix4d>(ftl::codecs::Channel::Pose);
}

const ftl::rgbd::Camera &Frame::getLeftCamera() const {
	return get<ftl::rgbd::Camera>(ftl::codecs::Channel::Calibration);
}

const ftl::rgbd::Camera &Frame::getRightCamera() const {
	return get<ftl::rgbd::Camera>(ftl::codecs::Channel::Calibration2);
}

void ftl::rgbd::Frame::setPose(const Eigen::Matrix4d &pose) {
	if (origin_) origin_->setPose(pose);
}

void ftl::rgbd::Frame::setLeftCamera(const ftl::rgbd::Camera &c) {
	if (origin_) origin_->setLeft(c);
}

void ftl::rgbd::Frame::setRightCamera(const ftl::rgbd::Camera &c) {
	if (origin_) origin_->setRight(c);
}

std::string ftl::rgbd::Frame::getConfigString() const {
	return get<nlohmann::json>(ftl::codecs::Channel::Configuration).dump();
}

