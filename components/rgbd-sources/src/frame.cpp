
#include <ftl/rgbd/frame.hpp>

using ftl::rgbd::Frame;
using ftl::rgbd::FrameState;
using ftl::codecs::Channels;
using ftl::codecs::Channel;
using ftl::rgbd::VideoData;

static cv::Mat none;
static cv::cuda::GpuMat noneGPU;

template <>
cv::Mat &VideoData::as<cv::Mat>() {
	if (isgpu) throw FTL_Error("Host request for GPU data without download");
	return host;
}

template <>
const cv::Mat &VideoData::as<cv::Mat>() const {
	if (isgpu) throw FTL_Error("Host request for GPU data without download");
	return host;
}

template <>
cv::cuda::GpuMat &VideoData::as<cv::cuda::GpuMat>() {
	if (!isgpu) throw FTL_Error("GPU request for Host data without upload");
	return gpu;
}

template <>
const cv::cuda::GpuMat &VideoData::as<cv::cuda::GpuMat>() const {
	if (!isgpu) throw FTL_Error("GPU request for Host data without upload");
	return gpu;
}

template <>
cv::Mat &VideoData::make<cv::Mat>() {
	isgpu = false;
	return host;
}

template <>
cv::cuda::GpuMat &VideoData::make<cv::cuda::GpuMat>() {
	isgpu = true;
	return gpu;
}

// =============================================================================

/*void Frame::reset() {
	origin_ = nullptr;
	channels_.clear();
	gpu_.clear();
	data_channels_.clear();
	for (size_t i=0u; i<Channels<0>::kMax; ++i) {
		data_[i].encoded.clear();
	}
}*/

/*void Frame::resetFull() {
	origin_ = nullptr;
	channels_.clear();
	gpu_.clear();
	for (size_t i=0u; i<Channels<0>::kMax; ++i) {
		data_[i].gpu = cv::cuda::GpuMat();
		data_[i].host = cv::Mat();
		data_[i].encoded.clear();
	}
}*/

void Frame::download(Channel c, cv::cuda::Stream stream) {
	download(Channels(c), stream);
}

void Frame::upload(Channel c, cv::cuda::Stream stream) {
	upload(Channels(c), stream);
}

void Frame::download(Channels<0> c, cv::cuda::Stream stream) {
	for (size_t i=0u; i<Channels<0>::kMax; ++i) {
		if (c.has(i) && hasChannel(static_cast<Channel>(i)) && isGPU(static_cast<Channel>(i))) {
			auto &data = getData(static_cast<Channel>(i));
			data.gpu.download(data.host, stream);
			data.isgpu = false;
		}
	}
}

void Frame::upload(Channels<0> c, cv::cuda::Stream stream) {
	for (size_t i=0u; i<Channels<0>::kMax; ++i) {
		if (c.has(i) && hasChannel(static_cast<Channel>(i)) && !isGPU(static_cast<Channel>(i))) {
			auto &data = getData(static_cast<Channel>(i));
			data.gpu.upload(data.host, stream);
			data.isgpu = true;
		}
	}
}

void Frame::pushPacket(ftl::codecs::Channel c, ftl::codecs::Packet &pkt) {
	if (hasChannel(c)) {
		auto &m1 = getData(c);
		m1.encoded.emplace_back() = std::move(pkt);
	} else {
		throw FTL_Error("Channel " << (int)c << " doesn't exist for packet push");
	}
}

const std::list<ftl::codecs::Packet> &Frame::getPackets(ftl::codecs::Channel c) const {
	if (!hasChannel(c)) {
		throw FTL_Error("Frame channel does not exist: " << (int)c);
	}

	auto &m1 = getData(c);
	return m1.encoded;
}

void Frame::mergeEncoding(ftl::rgbd::Frame &f) {
	//LOG(INFO) << "MERGE " << (unsigned int)f.channels_;
	for (auto c : getChannels()) {
		//if (!f.hasChannel(c)) f.create<cv::cuda::GpuMat>(c);
		if (f.hasChannel(c)) {
			auto &m1 = getData(c);
			auto &m2 = f.getData(c);
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

template <> cv::Mat &Frame::create(ftl::codecs::Channel c, const ftl::rgbd::FormatBase &f) {
	if (c == Channel::None) {
		throw FTL_Error("Cannot create a None channel");
	}
	
	create<cv::Mat>(c);
	auto &m = getData(c);

	m.encoded.clear();  // Remove all old encoded data

	if (!f.empty()) {
		m.host.create(f.size(), f.cvType);
	}

	return m.host;
}

template <> cv::cuda::GpuMat &Frame::create(ftl::codecs::Channel c, const ftl::rgbd::FormatBase &f) {
	if (c == Channel::None) {
		throw FTL_Error("Cannot create a None channel");
	}

	create<cv::cuda::GpuMat>(c);
	auto &m = getData(c);

	m.encoded.clear();  // Remove all old encoded data

	if (!f.empty()) {
		m.gpu.create(f.size(), f.cvType);
	}

	return m.gpu;
}

void Frame::clearPackets(ftl::codecs::Channel c) {
	auto &m = getData(c);
	m.encoded.clear();
}

void Frame::resetTexture(ftl::codecs::Channel c) {
	auto &m = getData(c);
	m.tex.free();
}
