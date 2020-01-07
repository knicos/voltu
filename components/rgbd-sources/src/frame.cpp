
#include <ftl/rgbd/frame.hpp>

using ftl::rgbd::Frame;
using ftl::codecs::Channels;
using ftl::codecs::Channel;

static cv::Mat none;
static cv::cuda::GpuMat noneGPU;

void Frame::reset() {
	channels_.clear();
	gpu_.clear();
	for (size_t i=0u; i<Channels::kMax; ++i) {
		data_[i].encoded.clear();
	}
}

void Frame::resetFull() {
	channels_.clear();
	gpu_.clear();
	for (size_t i=0u; i<Channels::kMax; ++i) {
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

void Frame::download(Channels c, cv::cuda::Stream stream) {
	for (size_t i=0u; i<Channels::kMax; ++i) {
		if (c.has(i) && channels_.has(i) && gpu_.has(i)) {
			data_[i].gpu.download(data_[i].host, stream);
			gpu_ -= i;
		}
	}
}

void Frame::upload(Channels c, cv::cuda::Stream stream) {
	for (size_t i=0u; i<Channels::kMax; ++i) {
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

bool Frame::empty(ftl::codecs::Channels channels) {
	for (auto c : channels) {
		if (empty(c)) return true;
	}
	return false;
}

void Frame::swapTo(ftl::codecs::Channels channels, Frame &f) {
	f.reset();

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

void Frame::copyTo(ftl::codecs::Channels channels, Frame &f) {
	f.reset();

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

