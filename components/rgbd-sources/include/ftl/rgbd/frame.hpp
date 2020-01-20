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
#include <ftl/rgbd/camera.hpp>
#include <ftl/codecs/codecs.hpp>
#include <ftl/codecs/packet.hpp>
#include <ftl/utility/vectorbuffer.hpp>

#include <ftl/cuda_common.hpp>

#include <type_traits>
#include <array>
#include <list>

#include <Eigen/Eigen>

namespace ftl {
namespace rgbd {

// TODO:	interpolation for scaling depends on channel type;
//			NN for depth/disparity/optflow, linear/cubic/etc. for RGB

class Frame;
class Source;

/**
 * Represent state that is persistent across frames. Such state may or may not
 * change from one frame to the next so a record of what has changed must be
 * kept. Changing state should be done at origin and not in the frame. State
 * that is marked as changed will then be send into a stream and the changed
 * status will be cleared, allowing data to only be sent/saved when actual
 * changes occur.
 */
class FrameState {
	public:
	FrameState();
	FrameState(FrameState &);
	FrameState(FrameState &&);

	/**
	 * Update the pose and mark as changed.
	 */
	void setPose(const Eigen::Matrix4d &pose);

	/**
	 * Update the left camera intrinsics and mark as changed.
	 */
	void setLeft(const ftl::rgbd::Camera &p);

	/**
	 * Update the right camera intrinsics and mark as changed.
	 */
	void setRight(const ftl::rgbd::Camera &p);

	/**
	 * Get the current camera pose.
	 */
	inline const Eigen::Matrix4d &getPose() const { return pose_; }

	/**
	 * Get the left camera intrinsics.
	 */
	inline const ftl::rgbd::Camera &getLeft() const { return camera_left_; }

	/**
	 * Get the right camera intrinsics.
	 */
	inline const ftl::rgbd::Camera &getRight() const { return camera_right_; }

	/**
	 * Get a modifiable pose reference that does not change the changed status.
	 * @attention Should only be used internally.
	 * @todo Make private eventually.
	 */
	inline Eigen::Matrix4d &getPose() { return pose_; }

	/**
	 * Get a modifiable left camera intrinsics reference that does not change
	 * the changed status. Modifications made using this will not be propagated.
	 * @attention Should only be used internally.
	 * @todo Make private eventually.
	 */
	inline ftl::rgbd::Camera &getLeft() { return camera_left_; }

	/**
	 * Get a modifiable right camera intrinsics reference that does not change
	 * the changed status. Modifications made using this will not be propagated.
	 * @attention Should only be used internally.
	 * @todo Make private eventually.
	 */
	inline ftl::rgbd::Camera &getRight() { return camera_right_; }

	/**
	 * Get a named config property.
	 */
	template <typename T>
	std::optional<T> get(const std::string &name) {
		try {
			return config_[name].get<T>();
		} catch (...) {
			return {};
		}
	}

	/**
	 * Set a named config property. Also makes state as changed to be resent.
	 */
	template <typename T>
	void set(const std::string &name, T value) {
		config_[name] = value;
		changed_ += ftl::codecs::Channel::Configuration;
	}

	inline const nlohmann::json &getConfig() const { return config_; }

	inline nlohmann::json &getConfig() { return config_; }

	/**
	 * Check if pose of intrinsics have been modified and not yet forwarded.
	 * Once forwarded through a pipeline / stream the changed status is cleared.
	 */
	inline bool hasChanged(ftl::codecs::Channel c) const { return changed_.has(c); }

	/**
	 * Copy assignment will clear the changed status of the original.
	 */
	FrameState &operator=(FrameState &);

	FrameState &operator=(FrameState &&);

	/**
	 * Clear the changed status to unchanged.
	 */
	inline void clear() { changed_.clear(); }

	private:
	Eigen::Matrix4d pose_;
	ftl::rgbd::Camera camera_left_;
	ftl::rgbd::Camera camera_right_;
	nlohmann::json config_;
	ftl::codecs::Channels<64> changed_;  // Have the state channels changed?
};

/**
 * Manage a set of image channels corresponding to a single camera frame.
 */
class Frame {
public:
	Frame() : origin_(nullptr) {}

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
	 * Perform a buffer swap of the selected channels. This is intended to be
	 * a copy from `this` to the passed frame object but by buffer swap
	 * instead of memory copy, meaning `this` may become invalid afterwards.
	 */
	void swapTo(ftl::codecs::Channels<0>, Frame &);

	void swapChannels(ftl::codecs::Channel, ftl::codecs::Channel);

	/**
	 * Does a host or device memory copy into the given frame.
	 */
	void copyTo(ftl::codecs::Channels<0>, Frame &);

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
	 * Set the value of a channel. Some channels should not be modified via the
	 * non-const get method, for example the data channels.
	 */
	template <typename T> void create(ftl::codecs::Channel channel, const T &value);

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
	 * Reset all channels without releasing memory.
	 */
	void reset();

	/**
	 * Reset all channels and release memory.
	 */
	void resetFull();

	/**
	 * Check if any specified channels are empty or missing.
	 */
	bool empty(ftl::codecs::Channels<0> c);

	/**
	 * Check if a specific channel is missing or has no memory allocated.
	 */
	inline bool empty(ftl::codecs::Channel c) {
		auto &m = _get(c);
		return !hasChannel(c) || (m.host.empty() && m.gpu.empty());
	}

	/**
	 * Is there valid data in channel (either host or gpu). This does not
	 * verify that any memory or data exists for the channel.
	 */
	inline bool hasChannel(ftl::codecs::Channel channel) const {
		int c = static_cast<int>(channel);
		if (c >= 64 && c <= 68) return true;
		else if (c >= 2048) return data_channels_.has(channel);
		else if (c >= 32) return false;
		else return channels_.has(channel);
	}

	/**
	 * Obtain a mask of all available channels in the frame.
	 */
	inline ftl::codecs::Channels<0> getChannels() const { return channels_; }
	inline ftl::codecs::Channels<0> getVideoChannels() const { return channels_; }

	inline ftl::codecs::Channels<2048> getDataChannels() const { return data_channels_; }

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
	 * Does this frame have new data for a channel. This is compared with a
	 * previous frame and always returns true for image data. It may return
	 * false for persistent state data (calibration, pose etc).
	 */
	inline bool hasChanged(ftl::codecs::Channel c) const {
		return (static_cast<int>(c) < 32) ? true : state_.hasChanged(c);
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

	template <typename T> void get(ftl::codecs::Channel channel, T &params) const;

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

	/**
	 * Get an existing CUDA texture object.
	 */
	template <typename T> const ftl::cuda::TextureObject<T> &getTexture(ftl::codecs::Channel) const;

	/**
	 * Get an existing CUDA texture object.
	 */
	template <typename T> ftl::cuda::TextureObject<T> &getTexture(ftl::codecs::Channel);

	/**
	 * Wrapper accessor function to get frame pose.
	 */
	const Eigen::Matrix4d &getPose() const;

	/**
	 * Change the pose of the origin state and mark as changed.
	 */
	void setPose(const Eigen::Matrix4d &pose);

	/**
	 * Wrapper to access left camera intrinsics channel.
	 */
	const ftl::rgbd::Camera &getLeftCamera() const;

	/**
	 * Wrapper to access right camera intrinsics channel.
	 */
	const ftl::rgbd::Camera &getRightCamera() const;

	/**
	 * Change left camera intrinsics in the origin state. This should send
	 * the changed parameters in reverse through a stream.
	 */
	void setLeftCamera(const ftl::rgbd::Camera &c);

	/**
	 * Change right camera intrinsics in the origin state. This should send
	 * the changed parameters in reverse through a stream.
	 */
	void setRightCamera(const ftl::rgbd::Camera &c);

	/**
	 * Dump the current frame config object to a json string.
	 */
	std::string getConfigString() const;

	/**
	 * Access the raw data channel vector object.
	 */
	const std::vector<unsigned char> &getRawData(ftl::codecs::Channel c) const;

	void createRawData(ftl::codecs::Channel c, const std::vector<unsigned char> &v);

	/**
	 * Wrapper to access a config property. If the property does not exist or
	 * is not of the requested type then the returned optional is false.
	 */
	template <typename T>
	std::optional<T> get(const std::string &name) { return state_.get<T>(name); }

	/**
	 * Modify a config property. This does not modify the origin config so
	 * will not get transmitted over the stream.
	 * @todo Modify origin to send backwards over a stream.
	 */
	template <typename T>
	void set(const std::string &name, T value) { state_.set(name, value); }

	/**
	 * Set the persistent state for the frame. This can only be done after
	 * construction or a reset. Multiple calls to this otherwise will throw
	 * an exception. The pointer must remain valid for the life of the frame.
	 */
	void setOrigin(ftl::rgbd::FrameState *state);

	/**
	 * Get the original frame state object. This can be a nullptr in some rare
	 * cases. When wishing to change state (pose, calibration etc) then those
	 * changes must be done on this origin, either directly or via wrappers.
	 */
	FrameState *origin() const { return origin_; }

private:
	struct ChannelData {
		ftl::cuda::TextureObjectBase tex;
		cv::Mat host;
		cv::cuda::GpuMat gpu;
		std::list<ftl::codecs::Packet> encoded;
	};

	std::array<ChannelData, ftl::codecs::Channels<0>::kMax> data_;
	std::unordered_map<int, std::vector<unsigned char>> data_data_;

	ftl::codecs::Channels<0> channels_;	// Does it have a channel
	ftl::codecs::Channels<0> gpu_;		// Is the channel on a GPU
	ftl::codecs::Channels<2048> data_channels_;

	// Persistent state
	FrameState state_;
	FrameState *origin_;

	/* Lookup internal state for a given channel. */
	inline ChannelData &_get(ftl::codecs::Channel c) { return data_[static_cast<unsigned int>(c)]; }
	inline const ChannelData &_get(ftl::codecs::Channel c) const { return data_[static_cast<unsigned int>(c)]; }
};

// Specialisations

template<> const cv::Mat& Frame::get(ftl::codecs::Channel channel) const;
template<> const cv::cuda::GpuMat& Frame::get(ftl::codecs::Channel channel) const;
template<> cv::Mat& Frame::get(ftl::codecs::Channel channel);
template<> cv::cuda::GpuMat& Frame::get(ftl::codecs::Channel channel);

//template<> const Eigen::Matrix4d &Frame::get(ftl::codecs::Channel channel) const;
template<> const ftl::rgbd::Camera &Frame::get(ftl::codecs::Channel channel) const;

// Default data channel implementation
template <typename T>
void Frame::get(ftl::codecs::Channel channel, T &params) const {
	if (static_cast<int>(channel) < static_cast<int>(ftl::codecs::Channel::Data)) throw ftl::exception("Cannot use generic type with non data channel");
	if (!hasChannel(channel)) throw ftl::exception("Data channel does not exist");

	const auto &i = data_data_.find(static_cast<int>(channel));
	if (i == data_data_.end()) throw ftl::exception("Data channel does not exist");

	auto unpacked = msgpack::unpack((const char*)(*i).second.data(), (*i).second.size());
	unpacked.get().convert(params);
}

template <> cv::Mat &Frame::create(ftl::codecs::Channel c, const ftl::rgbd::FormatBase &);
template <> cv::cuda::GpuMat &Frame::create(ftl::codecs::Channel c, const ftl::rgbd::FormatBase &);
template <> cv::Mat &Frame::create(ftl::codecs::Channel c);
template <> cv::cuda::GpuMat &Frame::create(ftl::codecs::Channel c);

template <typename T>
void Frame::create(ftl::codecs::Channel channel, const T &value) {
	if (static_cast<int>(channel) < static_cast<int>(ftl::codecs::Channel::Data)) throw ftl::exception("Cannot use generic type with non data channel");

	data_channels_ += channel;
	data_data_.insert({static_cast<int>(channel),{}});
	ftl::util::FTLVectorBuffer buf(data_data_[static_cast<int>(channel)]);
	msgpack::pack(buf, value);
}

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
		//LOG(INFO) << "Recreating texture object for '" << ftl::codecs::name(c) << "'";
		m.tex.free();
		m.tex = ftl::cuda::TextureObject<T>(m.gpu, interpolated);
	}

	return ftl::cuda::TextureObject<T>::cast(m.tex);
}

template <typename T>
ftl::cuda::TextureObject<T> &Frame::createTexture(ftl::codecs::Channel c, bool interpolated) {
	if (!channels_.has(c)) throw ftl::exception(ftl::Formatter() << "createTexture needs a format if the channel does not exist: " << (int)c);

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
		//LOG(INFO) << "Recreating texture object for '" << ftl::codecs::name(c) << "'.";
		m.tex.free();
		m.tex = ftl::cuda::TextureObject<T>(m.gpu, interpolated);
	}

	return ftl::cuda::TextureObject<T>::cast(m.tex);
}

}
}

#endif // _FTL_RGBD_FRAME_HPP_