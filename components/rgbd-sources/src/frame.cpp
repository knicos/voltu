
#include <ftl/rgbd/frame.hpp>

using ftl::rgbd::Frame;
using ftl::rgbd::Channels;
using ftl::rgbd::Channel;

static cv::Mat none;
static cv::cuda::GpuMat noneGPU;

void Frame::reset() {
	channels_.clear();
	gpu_.clear();
}

void Frame::download(Channel c, cv::cuda::Stream& stream) {
	download(Channels(c), stream);
}

void Frame::upload(Channel c, cv::cuda::Stream& stream) {
	upload(Channels(c), stream);
}

void Frame::download(Channels c, cv::cuda::Stream& stream) {
	for (size_t i=0u; i<Channels::kMax; ++i) {
		if (c.has(i) && channels_.has(i) && gpu_.has(i)) {
			data_[i].gpu.download(data_[i].host, stream);
			gpu_ -= i;
		}
	}
}

void Frame::upload(Channels c, cv::cuda::Stream& stream) {
	for (size_t i=0u; i<Channels::kMax; ++i) {
		if (c.has(i) && channels_.has(i) && !gpu_.has(i)) {
			data_[i].gpu.upload(data_[i].host, stream);
			gpu_ += i;
		}
	}
}

template<> cv::Mat& Frame::get(ftl::rgbd::Channel channel) {
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
		throw ftl::exception("Frame channel does not exist");
	}

	return _get(channel).host;
}

template<> cv::cuda::GpuMat& Frame::get(ftl::rgbd::Channel channel) {
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
		throw ftl::exception("Frame channel does not exist");
	}

	return _get(channel).gpu;
}

template<> const cv::Mat& Frame::get(ftl::rgbd::Channel channel) const {
	if (channel == Channel::None) {
		LOG(FATAL) << "Cannot get the None channel from a Frame";
	}

	if (isGPU(channel)) {
		LOG(FATAL) << "Getting GPU channel on CPU without explicit 'download'";
	}

	if (!channels_.has(channel)) throw ftl::exception("Frame channel does not exist");

	return _get(channel).host;
}

template<> const cv::cuda::GpuMat& Frame::get(ftl::rgbd::Channel channel) const {
	if (channel == Channel::None) {
		LOG(FATAL) << "Cannot get the None channel from a Frame";
	}

	if (isCPU(channel)) {
		LOG(FATAL) << "Getting CPU channel on GPU without explicit 'upload'";
	}

	// Add channel if not already there
	if (!channels_.has(channel)) {
		throw ftl::exception("Frame channel does not exist");
	}

	return _get(channel).gpu;
}

template <> cv::Mat &Frame::create(ftl::rgbd::Channel c, const ftl::rgbd::FormatBase &f) {
	if (c == Channel::None) {
		throw ftl::exception("Cannot create a None channel");
	}
	channels_ += c;
	gpu_ -= c;

	auto &m = _get(c).host;

	if (!f.empty()) {
		m.create(f.size(), f.cvType);
	}

	return m;
}

template <> cv::cuda::GpuMat &Frame::create(ftl::rgbd::Channel c, const ftl::rgbd::FormatBase &f) {
	if (c == Channel::None) {
		throw ftl::exception("Cannot create a None channel");
	}
	channels_ += c;
	gpu_ += c;

	auto &m = _get(c).gpu;

	if (!f.empty()) {
		m.create(f.size(), f.cvType);
	}

	return m;
}

template <> cv::Mat &Frame::create(ftl::rgbd::Channel c) {
	if (c == Channel::None) {
		throw ftl::exception("Cannot create a None channel");
	}
	channels_ += c;
	gpu_ -= c;

	auto &m = _get(c).host;
	return m;
}

template <> cv::cuda::GpuMat &Frame::create(ftl::rgbd::Channel c) {
	if (c == Channel::None) {
		throw ftl::exception("Cannot create a None channel");
	}
	channels_ += c;
	gpu_ += c;

	auto &m = _get(c).gpu;
	return m;
}

