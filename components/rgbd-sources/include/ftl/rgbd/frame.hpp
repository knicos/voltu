#pragma once
#ifndef _FTL_RGBD_FRAME_HPP_
#define _FTL_RGBD_FRAME_HPP_

#include <ftl/configuration.hpp>
#include <ftl/exception.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/cuda_stream_accessor.hpp>

#include <ftl/codecs/channels.hpp>
#include <ftl/rgbd/format.hpp>
#include <ftl/codecs/bitrates.hpp>

#include <ftl/cuda_common.hpp>

#include <type_traits>
#include <array>

namespace ftl {
namespace rgbd {

// TODO:	interpolation for scaling depends on channel type;
//			NN for depth/disparity/optflow, linear/cubic/etc. for RGB

class Frame;
class Source;

/**
 * Manage a set of image channels corresponding to a single camera frame.
 */
class Frame {
public:
	Frame() : src_(nullptr) {}
	explicit Frame(ftl::rgbd::Source *src) : src_(src) {}

	inline ftl::rgbd::Source *source() const { return src_; }

	// Prevent frame copy, instead use a move.
	//Frame(const Frame &)=delete;
	//Frame &operator=(const Frame &)=delete;

	void download(ftl::codecs::Channel c, cv::cuda::Stream stream);
	void upload(ftl::codecs::Channel c, cv::cuda::Stream stream);
	void download(ftl::codecs::Channels c, cv::cuda::Stream stream);
	void upload(ftl::codecs::Channels c, cv::cuda::Stream stream);

	inline void download(ftl::codecs::Channel c, cudaStream_t stream=0) { download(c, cv::cuda::StreamAccessor::wrapStream(stream)); };
	inline void upload(ftl::codecs::Channel c, cudaStream_t stream=0) { upload(c, cv::cuda::StreamAccessor::wrapStream(stream)); };
	inline void download(ftl::codecs::Channels c, cudaStream_t stream=0) { download(c, cv::cuda::StreamAccessor::wrapStream(stream)); };
	inline void upload(ftl::codecs::Channels c, cudaStream_t stream=0) { upload(c, cv::cuda::StreamAccessor::wrapStream(stream)); };

	/**
	 * Perform a buffer swap of the selected channels. This is intended to be
	 * a copy from `this` to the passed frame object but by buffer swap
	 * instead of memory copy, meaning `this` may become invalid afterwards.
	 */
	void swapTo(ftl::codecs::Channels, Frame &);

	void swapChannels(ftl::codecs::Channel, ftl::codecs::Channel);

	/**
	 * Does a host or device memory copy into the given frame.
	 */
	void copyTo(ftl::codecs::Channels, Frame &);

	/**
	 * Create a channel with a given format. This will discard any existing
	 * data associated with the channel and ensure all data structures and
	 * memory allocations match the new format.
	 */
	template <typename T> T &create(ftl::codecs::Channel c, const ftl::rgbd::FormatBase &f);

	/**
	 * Create a channel but without any format.
	 */
	template <typename T> T &create(ftl::codecs::Channel c);

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

	void resetTexture(ftl::codecs::Channel c);

	/**
	 * Reset all channels without releasing memory.
	 */
	void reset();

	bool empty(ftl::codecs::Channels c);

	inline bool empty(ftl::codecs::Channel c) {
		auto &m = _get(c);
		return !hasChannel(c) || (m.host.empty() && m.gpu.empty());
	}

	/**
	 * Is there valid data in channel (either host or gpu).
	 */
	inline bool hasChannel(ftl::codecs::Channel channel) const {
		return channels_.has(channel);
	}

	inline ftl::codecs::Channels getChannels() const { return channels_; }

	/**
	 * Is the channel data currently located on GPU. This also returns false if
	 * the channel does not exist.
	 */
	inline bool isGPU(ftl::codecs::Channel channel) const {
		return channels_.has(channel) && gpu_.has(channel);
	}

	/**
	 * Is the channel data currently located on CPU memory. This also returns
	 * false if the channel does not exist.
	 */
	inline bool isCPU(ftl::codecs::Channel channel) const {
		return channels_.has(channel) && !gpu_.has(channel);
	}

	/**
	 * Method to get reference to the channel content.
	 * @param	Channel type
	 * @return	Const reference to channel data
	 * 
	 * Result is valid only if hasChannel() is true. Host/Gpu transfer is
	 * performed, if necessary, but with a warning since an explicit upload or
	 * download should be used.
	 */
	template <typename T> const T& get(ftl::codecs::Channel channel) const;

	/**
	 * Method to get reference to the channel content.
	 * @param	Channel type
	 * @return	Reference to channel data
	 * 
	 * Result is valid only if hasChannel() is true. Host/Gpu transfer is
	 * performed, if necessary, but with a warning since an explicit upload or
	 * download should be used.
	 */
	template <typename T> T& get(ftl::codecs::Channel channel);

	template <typename T> const ftl::cuda::TextureObject<T> &getTexture(ftl::codecs::Channel) const;
	template <typename T> ftl::cuda::TextureObject<T> &getTexture(ftl::codecs::Channel);

private:
	struct ChannelData {
		ftl::cuda::TextureObjectBase tex;
		cv::Mat host;
		cv::cuda::GpuMat gpu;
	};

	std::array<ChannelData, ftl::codecs::Channels::kMax> data_;

	ftl::codecs::Channels channels_;	// Does it have a channel
	ftl::codecs::Channels gpu_;		// Is the channel on a GPU

	ftl::rgbd::Source *src_;

	inline ChannelData &_get(ftl::codecs::Channel c) { return data_[static_cast<unsigned int>(c)]; }
	inline const ChannelData &_get(ftl::codecs::Channel c) const { return data_[static_cast<unsigned int>(c)]; }
};

// Specialisations

template<> const cv::Mat& Frame::get(ftl::codecs::Channel channel) const;
template<> const cv::cuda::GpuMat& Frame::get(ftl::codecs::Channel channel) const;
template<> cv::Mat& Frame::get(ftl::codecs::Channel channel);
template<> cv::cuda::GpuMat& Frame::get(ftl::codecs::Channel channel);

template <> cv::Mat &Frame::create(ftl::codecs::Channel c, const ftl::rgbd::FormatBase &);
template <> cv::cuda::GpuMat &Frame::create(ftl::codecs::Channel c, const ftl::rgbd::FormatBase &);
template <> cv::Mat &Frame::create(ftl::codecs::Channel c);
template <> cv::cuda::GpuMat &Frame::create(ftl::codecs::Channel c);

template <typename T>
ftl::cuda::TextureObject<T> &Frame::getTexture(ftl::codecs::Channel c) {
	if (!channels_.has(c)) throw ftl::exception(ftl::Formatter() << "Texture channel does not exist: " << (int)c);
	if (!gpu_.has(c)) throw ftl::exception("Texture channel is not on GPU");

	auto &m = _get(c);

	if (m.tex.cvType() != ftl::traits::OpenCVType<T>::value || m.tex.width() != m.gpu.cols || m.tex.height() != m.gpu.rows || m.gpu.type() != m.tex.cvType()) {
		LOG(ERROR) << "Texture has not been created for channel = " << (int)c;
		throw ftl::exception("Texture has not been created properly for this channel");
	}

	return ftl::cuda::TextureObject<T>::cast(m.tex);
}

template <typename T>
ftl::cuda::TextureObject<T> &Frame::createTexture(ftl::codecs::Channel c, const ftl::rgbd::Format<T> &f, bool interpolated) {
	if (!channels_.has(c)) channels_ += c;
	if (!gpu_.has(c)) gpu_ += c;

	auto &m = _get(c);

	if (f.empty()) {
		throw ftl::exception("createTexture needs a non-empty format");
	} else {
		m.gpu.create(f.size(), f.cvType);
	}

	if (m.gpu.type() != ftl::traits::OpenCVType<T>::value) {
		LOG(ERROR) << "Texture type mismatch: " << (int)c << " " << m.gpu.type() << " != " << ftl::traits::OpenCVType<T>::value;
		throw ftl::exception("Texture type does not match underlying data type");
	}

	// TODO: Check tex cvType

	if (m.tex.devicePtr() == nullptr) {
		//LOG(INFO) << "Creating texture object";
		m.tex = ftl::cuda::TextureObject<T>(m.gpu, interpolated);
	} else if (m.tex.cvType() != ftl::traits::OpenCVType<T>::value || m.tex.width() != m.gpu.cols || m.tex.height() != m.gpu.rows) {
		LOG(INFO) << "Recreating texture object for '" << ftl::codecs::name(c) << "'";
		m.tex.free();
		m.tex = ftl::cuda::TextureObject<T>(m.gpu, interpolated);
	}

	return ftl::cuda::TextureObject<T>::cast(m.tex);
}

template <typename T>
ftl::cuda::TextureObject<T> &Frame::createTexture(ftl::codecs::Channel c, bool interpolated) {
	if (!channels_.has(c)) throw ftl::exception("createTexture needs a format if the channel does not exist");

	auto &m = _get(c);

	if (isCPU(c) && !m.host.empty()) {
		m.gpu.create(m.host.size(), m.host.type());
		// TODO: Should this upload to GPU or not?
		//gpu_ += c;
	} else if (isCPU(c) || (isGPU(c) && m.gpu.empty())) {
		throw ftl::exception("createTexture needs a format if no memory is allocated");
	}

	if (m.gpu.type() != ftl::traits::OpenCVType<T>::value) {
		LOG(ERROR) << "Texture type mismatch: " << (int)c << " " << m.gpu.type() << " != " << ftl::traits::OpenCVType<T>::value;
		throw ftl::exception("Texture type does not match underlying data type");
	}

	// TODO: Check tex cvType

	if (m.tex.devicePtr() == nullptr) {
		//LOG(INFO) << "Creating texture object";
		m.tex = ftl::cuda::TextureObject<T>(m.gpu, interpolated);
	} else if (m.tex.cvType() != ftl::traits::OpenCVType<T>::value || m.tex.width() != m.gpu.cols || m.tex.height() != m.gpu.rows || m.tex.devicePtr() != m.gpu.data) {
		LOG(INFO) << "Recreating texture object for '" << ftl::codecs::name(c) << "'.";
		m.tex.free();
		m.tex = ftl::cuda::TextureObject<T>(m.gpu, interpolated);
	}

	return ftl::cuda::TextureObject<T>::cast(m.tex);
}

}
}

#endif // _FTL_RGBD_FRAME_HPP_