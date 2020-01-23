#pragma once
#ifndef _FTL_DATA_FRAME_HPP_
#define _FTL_DATA_FRAME_HPP_

#include <ftl/configuration.hpp>
#include <ftl/exception.hpp>

#include <ftl/codecs/channels.hpp>
#include <ftl/codecs/codecs.hpp>
//#include <ftl/codecs/packet.hpp>
#include <ftl/utility/vectorbuffer.hpp>

#include <type_traits>
#include <array>
//#include <list>
#include <unordered_map>

#include <Eigen/Eigen>

namespace ftl {
namespace data {

/**
 * Manage a set of channels corresponding to a single frame. There are three
 * kinds of channel in a frame: 1) the data type of interest (DoI)
 * (eg. audio, video, etc), 2) Persistent state and 3) Generic meta data.
 * The DoI is a template arg and could be in any form. Different DoIs will use
 * different frame instances, ie. audio and video frame types. Persistent state
 * may or may not change between frames but is always available. Finally,
 * generic data is a small amount of information about the primary data that may
 * or may not exist each frame, and is not required to exist.
 * 
 * There is no specification for frame rates, intervals or synchronisation at
 * this level. A frame is a quantum of data of any temporal size which can be
 * added to a FrameSet to be synchronised with other frames.
 * 
 * Use this template class either by inheriting it or just by providing the
 * template arguments. It is not abstract and can work directly.
 * 
 * The template DATA parameter must be a class or struct that implements three
 * methods: 1) `const T& at<T>()` to cast to const type, 2) `T& at<T>()` to cast
 * to non-const type, and 3) `T& make<T>() to create data as a type.
 * 
 * The STATE parameter must be an instance of `ftl::data::FrameState`.
 * 
 * @see ftl::data::FrameState
 * @see ftl::data::FrameSet
 * @see ftl::rgbd::FrameState
 * @see ftl::rgbd::Frame
 */
template <int BASE, int N, typename STATE, typename DATA>
class Frame {
	static_assert(N <= ftl::codecs::Channels<BASE>::kMax, "Too many channels requested");

public:
	Frame() : origin_(nullptr) {}

	// Prevent frame copy, instead use a move.
	//Frame(const Frame &)=delete;
	//Frame &operator=(const Frame &)=delete;

	/**
	 * Perform a buffer swap of the selected channels. This is intended to be
	 * a copy from `this` to the passed frame object but by buffer swap
	 * instead of memory copy, meaning `this` may become invalid afterwards.
	 */
	void swapTo(ftl::codecs::Channels<BASE>, Frame &);

	void swapTo(Frame &);

	void swapChannels(ftl::codecs::Channel, ftl::codecs::Channel);

	/**
	 * Does a host or device memory copy into the given frame.
	 */
	void copyTo(ftl::codecs::Channels<BASE>, Frame &);

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
	 * Append encoded data for a channel. This will move the data, invalidating
	 * the original packet structure. It is to be used to allow data that is
	 * already encoded to be transmitted or saved again without re-encoding.
	 * A called to `create` will clear all encoded data for that channel.
	 */
	//void pushPacket(ftl::codecs::Channel c, ftl::codecs::Packet &pkt);

	/**
	 * Obtain a list of any existing encodings for this channel.
	 */
	//const std::list<ftl::codecs::Packet> &getPackets(ftl::codecs::Channel c) const;

	/**
	 * Clear any existing encoded packets. Used when the channel data is
	 * modified and the encodings are therefore out-of-date.
	 */
	//void clearPackets(ftl::codecs::Channel c);

	/**
	 * Reset all channels without releasing memory.
	 */
	void reset();

	/**
	 * Reset all channels and release memory.
	 */
	//void resetFull();

	/**
	 * Is there valid data in channel (either host or gpu). This does not
	 * verify that any memory or data exists for the channel.
	 */
	bool hasChannel(ftl::codecs::Channel channel) const {
		int c = static_cast<int>(channel);
		if (c >= 64 && c <= 68) return true;
		else if (c >= 2048) return data_channels_.has(channel);
		else if (c < BASE || c >= BASE+N) return false;
		else return channels_.has(channel);
	}

	/**
	 * Obtain a mask of all available channels in the frame.
	 */
	inline ftl::codecs::Channels<BASE> getChannels() const { return channels_; }

	inline ftl::codecs::Channels<2048> getDataChannels() const { return data_channels_; }

	/**
	 * Does this frame have new data for a channel. This is compared with a
	 * previous frame and always returns true for image data. It may return
	 * false for persistent state data (calibration, pose etc).
	 */
	inline bool hasChanged(ftl::codecs::Channel c) const {
		return (static_cast<int>(c) < 64) ? true : state_.hasChanged(c);
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
	 * Get the data from a data channel. This only works for the data channels
	 * and will throw an exception with any others.
	 */
	template <typename T> void get(ftl::codecs::Channel channel, T &params) const;

	/**
	 * Method to get reference to the channel content.
	 * @param	Channel type
	 * @return	Reference to channel data
	 * 
	 * Result is valid only if hasChannel() is true.
	 */
	template <typename T> T& get(ftl::codecs::Channel channel);

	/**
	 * Wrapper accessor function to get frame pose.
	 */
	const Eigen::Matrix4d &getPose() const;

	/**
	 * Change the pose of the origin state and mark as changed.
	 */
	void setPose(const Eigen::Matrix4d &pose);

	/**
	 * Wrapper to access left settings channel.
	 */
	const typename STATE::Settings &getSettings() const;

	const typename STATE::Settings &getLeft() const;
	const typename STATE::Settings &getRight() const;

	void setLeft(const typename STATE::Settings &);
	void setRight(const typename STATE::Settings &);

	/**
	 * Change left settings in the origin state. This should send
	 * the changed parameters in reverse through a stream.
	 */
	void setSettings(const typename STATE::Settings &c);

	/**
	 * Dump the current frame config object to a json string.
	 */
	std::string getConfigString() const;

	/**
	 * Access the raw data channel vector object.
	 */
	const std::vector<unsigned char> &getRawData(ftl::codecs::Channel c) const;

	/**
	 * Provide raw data for a data channel.
	 */
	void createRawData(ftl::codecs::Channel c, const std::vector<unsigned char> &v);

	/**
	 * Wrapper to access a config property. If the property does not exist or
	 * is not of the requested type then the returned optional is false.
	 */
	template <class T>
	std::optional<T> get(const std::string &name) { return state_.template get<T>(name); }

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
	void setOrigin(STATE *state);

	/**
	 * Get the original frame state object. This can be a nullptr in some rare
	 * cases. When wishing to change state (pose, calibration etc) then those
	 * changes must be done on this origin, either directly or via wrappers.
	 */
	STATE *origin() const { return origin_; }

	typedef STATE State;

protected:
	/* Lookup internal state for a given channel. */
	inline DATA &getData(ftl::codecs::Channel c) { return data_[static_cast<unsigned int>(c)-BASE]; }
	inline const DATA &getData(ftl::codecs::Channel c) const { return data_[static_cast<unsigned int>(c)-BASE]; }

private:
	std::array<DATA, N> data_;

	std::unordered_map<int, std::vector<unsigned char>> data_data_;

	ftl::codecs::Channels<BASE> channels_;	// Does it have a channel
	ftl::codecs::Channels<2048> data_channels_;

	// Persistent state
	STATE state_;
	STATE *origin_;
};

}
}

// ==== Implementations ========================================================

template <int BASE, int N, typename STATE, typename DATA>
void ftl::data::Frame<BASE,N,STATE,DATA>::reset() {
	origin_ = nullptr;
	channels_.clear();
	data_channels_.clear();
	for (size_t i=0u; i<ftl::codecs::Channels<BASE>::kMax; ++i) {
		data_[i].reset();
	}
}

template <int BASE, int N, typename STATE, typename DATA>
void ftl::data::Frame<BASE,N,STATE,DATA>::swapTo(ftl::codecs::Channels<BASE> channels, Frame<BASE,N,STATE,DATA> &f) {
	f.reset();
	f.origin_ = origin_;
	f.state_ = state_;

	// For all channels in this frame object
	for (auto c : channels_) {
		// Should we swap this channel?
		if (channels.has(c)) {
			f.channels_ += c;
			std::swap(f.getData(c),getData(c));
		}
	}

	f.data_data_ = std::move(data_data_);
	f.data_channels_ = data_channels_;
	data_channels_.clear();
	channels_.clear();
}

template <int BASE, int N, typename STATE, typename DATA>
void ftl::data::Frame<BASE,N,STATE,DATA>::swapTo(Frame<BASE,N,STATE,DATA> &f) {
	swapTo(ftl::codecs::Channels<BASE>::All(), f);
}

template <int BASE, int N, typename STATE, typename DATA>
void ftl::data::Frame<BASE,N,STATE,DATA>::swapChannels(ftl::codecs::Channel a, ftl::codecs::Channel b) {
	auto &m1 = getData(a);
	auto &m2 = getData(b);

	auto temp = std::move(m2);
	m2 = std::move(m1);
	m1 = std::move(temp);
}

template <int BASE, int N, typename STATE, typename DATA>
void ftl::data::Frame<BASE,N,STATE,DATA>::copyTo(ftl::codecs::Channels<BASE> channels, Frame<BASE,N,STATE,DATA> &f) {
	f.reset();
	f.origin_ = origin_;
	f.state_ = state_;

	// For all channels in this frame object
	for (auto c : channels_) {
		// Should we copy this channel?
		if (channels.has(c)) {
			f.channels_ += c;
			f.getData(c) = getData(c);
		}
	}

	f.data_data_ = data_data_;
	f.data_channels_ = data_channels_;
}

template <int BASE, int N, typename STATE, typename DATA>
template <typename T>
T& ftl::data::Frame<BASE,N,STATE,DATA>::get(ftl::codecs::Channel channel) {
	if (channel == ftl::codecs::Channel::None) {
		throw ftl::exception("Attempting to get channel 'None'");
	}

	// Add channel if not already there
	if (!channels_.has(channel)) {
		throw ftl::exception(ftl::Formatter() << "Frame channel does not exist: " << (int)channel);
	}

	return getData(channel).template as<T>();
}

template <int BASE, int N, typename STATE, typename DATA>
template <typename T>
const T& ftl::data::Frame<BASE,N,STATE,DATA>::get(ftl::codecs::Channel channel) const {
	if (channel == ftl::codecs::Channel::None) {
		throw ftl::exception("Attempting to get channel 'None'");
	} else if (channel == ftl::codecs::Channel::Pose) {
		return state_.template as<T,ftl::codecs::Channel::Pose>();
	} else if (channel == ftl::codecs::Channel::Calibration) {
		return state_.template as<T,ftl::codecs::Channel::Calibration>();
	} else if (channel == ftl::codecs::Channel::Calibration2) {
		return state_.template as<T,ftl::codecs::Channel::Calibration2>();
	} else if (channel == ftl::codecs::Channel::Configuration) {
		return state_.template as<T,ftl::codecs::Channel::Configuration>();
	}

	// Add channel if not already there
	if (!channels_.has(channel)) {
		throw ftl::exception(ftl::Formatter() << "Frame channel does not exist: " << (int)channel);
	}

	return getData(channel).template as<T>();
}

// Default data channel implementation
template <int BASE, int N, typename STATE, typename DATA>
template <typename T>
void ftl::data::Frame<BASE,N,STATE,DATA>::get(ftl::codecs::Channel channel, T &params) const {
	if (static_cast<int>(channel) < static_cast<int>(ftl::codecs::Channel::Data)) throw ftl::exception("Cannot use generic type with non data channel");
	if (!hasChannel(channel)) throw ftl::exception("Data channel does not exist");

	const auto &i = data_data_.find(static_cast<int>(channel));
	if (i == data_data_.end()) throw ftl::exception("Data channel does not exist");

	auto unpacked = msgpack::unpack((const char*)(*i).second.data(), (*i).second.size());
	unpacked.get().convert(params);
}

template <int BASE, int N, typename STATE, typename DATA>
template <typename T>
T &ftl::data::Frame<BASE,N,STATE,DATA>::create(ftl::codecs::Channel c) {
	if (c == ftl::codecs::Channel::None) {
		throw ftl::exception("Cannot create a None channel");
	}
	channels_ += c;

	auto &m = getData(c);
	return m.template make<T>();
}

template <int BASE, int N, typename STATE, typename DATA>
template <typename T>
void ftl::data::Frame<BASE,N,STATE,DATA>::create(ftl::codecs::Channel channel, const T &value) {
	if (static_cast<int>(channel) < static_cast<int>(ftl::codecs::Channel::Data)) throw ftl::exception("Cannot use generic type with non data channel");

	data_channels_ += channel;

	auto &v = *std::get<0>(data_data_.insert({static_cast<int>(channel),{}}));
	v.second.resize(0);
	ftl::util::FTLVectorBuffer buf(v.second);
	msgpack::pack(buf, value);
}

template <int BASE, int N, typename STATE, typename DATA>
void ftl::data::Frame<BASE,N,STATE,DATA>::setOrigin(STATE *state) {
	if (origin_ != nullptr) {
		throw ftl::exception("Can only set origin once after reset");
	}

	origin_ = state;
	state_ = *state;
}

template <int BASE, int N, typename STATE, typename DATA>
const Eigen::Matrix4d &ftl::data::Frame<BASE,N,STATE,DATA>::getPose() const {
	return get<Eigen::Matrix4d>(ftl::codecs::Channel::Pose);
}

template <int BASE, int N, typename STATE, typename DATA>
const typename STATE::Settings &ftl::data::Frame<BASE,N,STATE,DATA>::getLeft() const {
	return get<typename STATE::Settings>(ftl::codecs::Channel::Calibration);
}

template <int BASE, int N, typename STATE, typename DATA>
const typename STATE::Settings &ftl::data::Frame<BASE,N,STATE,DATA>::getRight() const {
	return get<typename STATE::Settings>(ftl::codecs::Channel::Calibration2);
}

template <int BASE, int N, typename STATE, typename DATA>
void ftl::data::Frame<BASE,N,STATE,DATA>::setPose(const Eigen::Matrix4d &pose) {
	if (origin_) origin_->setPose(pose);
}

template <int BASE, int N, typename STATE, typename DATA>
void ftl::data::Frame<BASE,N,STATE,DATA>::setLeft(const typename STATE::Settings &c) {
	if (origin_) origin_->setLeft(c);
}

template <int BASE, int N, typename STATE, typename DATA>
void ftl::data::Frame<BASE,N,STATE,DATA>::setRight(const typename STATE::Settings &c) {
	if (origin_) origin_->setRight(c);
}

template <int BASE, int N, typename STATE, typename DATA>
std::string ftl::data::Frame<BASE,N,STATE,DATA>::getConfigString() const {
	return get<nlohmann::json>(ftl::codecs::Channel::Configuration).dump();
}

template <int BASE, int N, typename STATE, typename DATA>
const std::vector<unsigned char> &ftl::data::Frame<BASE,N,STATE,DATA>::getRawData(ftl::codecs::Channel channel) const {
	if (static_cast<int>(channel) < static_cast<int>(ftl::codecs::Channel::Data)) throw ftl::exception("Non data channel");
	if (!hasChannel(channel)) throw ftl::exception("Data channel does not exist");

	return data_data_.at(static_cast<int>(channel));
}

template <int BASE, int N, typename STATE, typename DATA>
void ftl::data::Frame<BASE,N,STATE,DATA>::createRawData(ftl::codecs::Channel c, const std::vector<unsigned char> &v) {
	data_data_.insert({static_cast<int>(c), v});
	data_channels_ += c;
}

#endif // _FTL_DATA_FRAME_HPP_
