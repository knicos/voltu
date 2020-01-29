#pragma once
#ifndef _FTL_RGBD_FRAME_HPP_
#define _FTL_RGBD_FRAME_HPP_

#include <ftl/configuration.hpp>
#include <ftl/exception.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/cuda_stream_accessor.hpp>

#include <ftl/data/frame.hpp>

#include <ftl/codecs/channels.hpp>
#include <ftl/rgbd/format.hpp>
#include <ftl/rgbd/camera.hpp>
#include <ftl/codecs/codecs.hpp>
#include <ftl/codecs/packet.hpp>
#include <ftl/utility/vectorbuffer.hpp>
#include <ftl/data/framestate.hpp>
#include <ftl/cuda_common.hpp>

#include <type_traits>
#include <array>
#include <list>

#include <Eigen/Eigen>

namespace ftl {
namespace rgbd {

typedef ftl::data::FrameState<ftl::rgbd::Camera,2> FrameState;

struct VideoData {
	ftl::cuda::TextureObjectBase tex;
	cv::cuda::GpuMat gpu;
	cv::Mat host;
	bool isgpu;
	std::list<ftl::codecs::Packet> encoded;

	template <typename T>
	T &as() {
		throw FTL_Error("Unsupported type for Video data channel");
	};

	template <typename T>
	const T &as() const {
		throw FTL_Error("Unsupported type for Video data channel");
	};

	template <typename T>
	T &make() {
		throw FTL_Error("Unsupported type for Video data channel");
	};

	inline void reset() {
		encoded.clear();
	}
};

// Specialisations for cv mat types
template <> cv::Mat &VideoData::as<cv::Mat>();
template <> const cv::Mat &VideoData::as<cv::Mat>() const;
template <> cv::cuda::GpuMat &VideoData::as<cv::cuda::GpuMat>();
template <> const cv::cuda::GpuMat &VideoData::as<cv::cuda::GpuMat>() const;

template <> cv::Mat &VideoData::make<cv::Mat>();
template <> cv::cuda::GpuMat &VideoData::make<cv::cuda::GpuMat>();

/**
 * Manage a set of image channels corresponding to a single camera frame.
 */
class Frame : public ftl::data::Frame<0,32,ftl::rgbd::FrameState,VideoData> {
//class Frame {
public:
	using ftl::data::Frame<0,32,ftl::rgbd::FrameState,VideoData>::create;

	Frame() {}

	// Prevent frame copy, instead use a move.
	//Frame(const Frame &)=delete;
	//Frame &operator=(const Frame &)=delete;

	void download(ftl::codecs::Channel c, cv::cuda::Stream stream);
	void upload(ftl::codecs::Channel c, cv::cuda::Stream stream);
	void download(ftl::codecs::Channels<0> c, cv::cuda::Stream stream);
	void upload(ftl::codecs::Channels<0> c, cv::cuda::Stream stream);

	inline void download(ftl::codecs::Channel c, cudaStream_t stream=0) { download(c, cv::cuda::StreamAccessor::wrapStream(stream)); };
	inline void upload(ftl::codecs::Channel c, cudaStream_t stream=0) { upload(c, cv::cuda::StreamAccessor::wrapStream(stream)); };
	inline void download(const ftl::codecs::Channels<0> &c, cudaStream_t stream=0) { download(c, cv::cuda::StreamAccessor::wrapStream(stream)); };
	inline void upload(const ftl::codecs::Channels<0> &c, cudaStream_t stream=0) { upload(c, cv::cuda::StreamAccessor::wrapStream(stream)); };

	/**
	 * Get an existing CUDA texture object.
	 */
	template <typename T> const ftl::cuda::TextureObject<T> &getTexture(ftl::codecs::Channel) const;

	/**
	 * Get an existing CUDA texture object.
	 */
	template <typename T> ftl::cuda::TextureObject<T> &getTexture(ftl::codecs::Channel);

	/**
	 * Create a channel with a given format. This will discard any existing
	 * data associated with the channel and ensure all data structures and
	 * memory allocations match the new format.
	 */
	template <typename T> T &create(ftl::codecs::Channel c, const ftl::rgbd::FormatBase &f);

	/**
	 * Create a CUDA texture object for a channel. This version takes a format
	 * argument to also create (or recreate) the associated GpuMat.
	 */
	template <typename T>
	ftl::cuda::TextureObject<T> &createTexture(ftl::codecs::Channel c, const ftl::rgbd::Format<T> &f, bool interpolated=false);

	/**
	 * Create a CUDA texture object for a channel. With this version the GpuMat
	 * must already exist and be of the correct type.
	 */
	template <typename T>
	ftl::cuda::TextureObject<T> &createTexture(ftl::codecs::Channel c, bool interpolated=false);

	/**
	 * Append encoded data for a channel. This will move the data, invalidating
	 * the original packet structure. It is to be used to allow data that is
	 * already encoded to be transmitted or saved again without re-encoding.
	 * A called to `create` will clear all encoded data for that channel.
	 */
	void pushPacket(ftl::codecs::Channel c, ftl::codecs::Packet &pkt);

	/**
	 * Obtain a list of any existing encodings for this channel.
	 */
	const std::list<ftl::codecs::Packet> &getPackets(ftl::codecs::Channel c) const;

	/**
	 * Clear any existing encoded packets. Used when the channel data is
	 * modified and the encodings are therefore out-of-date.
	 */
	void clearPackets(ftl::codecs::Channel c);

	/**
	 * Packets from multiple frames are merged together in sequence. An example
	 * case is if a frame gets dropped but the original encoding is inter-frame
	 * and hence still requires the dropped frames encoding data.
	 */
	void mergeEncoding(ftl::rgbd::Frame &f);

	void resetTexture(ftl::codecs::Channel c);

	/**
	 * Check if any specified channels are empty or missing.
	 */
	bool empty(ftl::codecs::Channels<0> c);

	/**
	 * Check if a specific channel is missing or has no memory allocated.
	 */
	inline bool empty(ftl::codecs::Channel c) {
		auto &m = getData(c);
		return !hasChannel(c) || (m.host.empty() && m.gpu.empty());
	}

	/**
	 * Obtain a mask of all available channels in the frame.
	 */
	inline ftl::codecs::Channels<0> getVideoChannels() const { return getChannels(); }

	inline const ftl::rgbd::Camera &getLeftCamera() const { return getLeft(); }
	inline const ftl::rgbd::Camera &getRightCamera() const { return getRight(); }

	/**
	 * Is the channel data currently located on GPU. This also returns false if
	 * the channel does not exist.
	 */
	inline bool isGPU(ftl::codecs::Channel channel) const {
		return hasChannel(channel) && getData(channel).isgpu;
	}

	/**
	 * Is the channel data currently located on CPU memory. This also returns
	 * false if the channel does not exist.
	 */
	inline bool isCPU(ftl::codecs::Channel channel) const {
		return hasChannel(channel) && !getData(channel).isgpu;
	}
};

// Specialisations

template <> cv::Mat &Frame::create(ftl::codecs::Channel c, const ftl::rgbd::FormatBase &);
template <> cv::cuda::GpuMat &Frame::create(ftl::codecs::Channel c, const ftl::rgbd::FormatBase &);

template <typename T>
ftl::cuda::TextureObject<T> &Frame::getTexture(ftl::codecs::Channel c) {
	if (!hasChannel(c)) throw FTL_Error("Texture channel does not exist: " << (int)c);

	auto &m = getData(c);
	if (!m.isgpu) throw FTL_Error("Texture channel is not on GPU");

	if (m.tex.cvType() != ftl::traits::OpenCVType<T>::value || m.tex.width() != m.gpu.cols || m.tex.height() != m.gpu.rows || m.gpu.type() != m.tex.cvType()) {
		throw FTL_Error("Texture has not been created properly for this channel: " << (int)c);
	}

	return ftl::cuda::TextureObject<T>::cast(m.tex);
}

template <typename T>
ftl::cuda::TextureObject<T> &Frame::createTexture(ftl::codecs::Channel c, const ftl::rgbd::Format<T> &f, bool interpolated) {
	//if (!hasChannel(c)) channels_ += c;
	//using ftl::data::Frame<0,32,ftl::rgbd::FrameState,VideoData>::create;

	ftl::data::Frame<0,32,ftl::rgbd::FrameState,VideoData>::create<cv::cuda::GpuMat>(c);
	auto &m = getData(c);

	if (f.empty()) {
		throw FTL_Error("createTexture needs a non-empty format");
	} else {
		m.gpu.create(f.size(), f.cvType);
	}

	if (m.gpu.type() != ftl::traits::OpenCVType<T>::value) {
		throw FTL_Error("Texture type mismatch: " << (int)c << " " << m.gpu.type() << " != " << ftl::traits::OpenCVType<T>::value);
	}

	// TODO: Check tex cvType

	if (m.tex.devicePtr() == nullptr) {
		//LOG(INFO) << "Creating texture object";
		m.tex = ftl::cuda::TextureObject<T>(m.gpu, interpolated);
	} else if (m.tex.cvType() != ftl::traits::OpenCVType<T>::value || m.tex.width() != m.gpu.cols || m.tex.height() != m.gpu.rows) {
		//LOG(INFO) << "Recreating texture object for '" << ftl::codecs::name(c) << "'";
		m.tex.free();
		m.tex = ftl::cuda::TextureObject<T>(m.gpu, interpolated);
	}

	return ftl::cuda::TextureObject<T>::cast(m.tex);
}

template <typename T>
ftl::cuda::TextureObject<T> &Frame::createTexture(ftl::codecs::Channel c, bool interpolated) {
	if (!hasChannel(c)) throw FTL_Error("createTexture needs a format if the channel does not exist: " << (int)c);

	auto &m = getData(c);

	if (!m.isgpu && !m.host.empty()) {
		m.gpu.create(m.host.size(), m.host.type());
		// TODO: Should this upload to GPU or not?
		//gpu_ += c;
	} else if (!m.isgpu || (m.isgpu && m.gpu.empty())) {
		throw FTL_Error("createTexture needs a format if no memory is allocated");
	}

	if (m.gpu.type() != ftl::traits::OpenCVType<T>::value) {
		throw FTL_Error("Texture type mismatch: " << (int)c << " " << m.gpu.type() << " != " << ftl::traits::OpenCVType<T>::value);
	}

	// TODO: Check tex cvType

	if (m.tex.devicePtr() == nullptr) {
		//LOG(INFO) << "Creating texture object";
		m.tex = ftl::cuda::TextureObject<T>(m.gpu, interpolated);
	} else if (m.tex.cvType() != ftl::traits::OpenCVType<T>::value || m.tex.width() != m.gpu.cols || m.tex.height() != m.gpu.rows || m.tex.devicePtr() != m.gpu.data) {
		//LOG(INFO) << "Recreating texture object for '" << ftl::codecs::name(c) << "'.";
		m.tex.free();
		m.tex = ftl::cuda::TextureObject<T>(m.gpu, interpolated);
	}

	return ftl::cuda::TextureObject<T>::cast(m.tex);
}

}
}

#endif // _FTL_RGBD_FRAME_HPP_