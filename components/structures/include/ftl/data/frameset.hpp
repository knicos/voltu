#ifndef _FTL_DATA_FRAMESET_HPP_
#define _FTL_DATA_FRAMESET_HPP_

#include <ftl/threads.hpp>
#include <ftl/timer.hpp>
#include <ftl/data/frame.hpp>
#include <functional>

//#include <opencv2/opencv.hpp>
#include <vector>

namespace ftl {
namespace data {

// Allows a latency of 20 frames maximum
//static const size_t kMaxFramesets = 15;
static const size_t kMaxFramesInSet = 32;

enum class FSFlag : int {
	STALE = 0,
	PARTIAL = 1
};

/**
 * Represents a set of synchronised frames, each with two channels. This is
 * used to collect all frames from multiple computers that have the same
 * timestamp.
 */
template <typename FRAME>
class FrameSet {
	public:

	int id=0;
	int64_t timestamp;				// Millisecond timestamp of all frames
	int64_t originClockDelta;
	std::vector<FRAME> frames;
	std::atomic<int> count;				// Number of valid frames
	std::atomic<unsigned int> mask;		// Mask of all sources that contributed
	//bool stale;						// True if buffers have been invalidated
	SHARED_MUTEX mtx;

	Eigen::Matrix4d pose;  // Set to identity by default.

	inline int64_t localTimestamp() const { return timestamp + originClockDelta; }

	void set(FSFlag f) { flags_ |= (1 << static_cast<int>(f)); }
	void clear(FSFlag f) { flags_ &= ~(1 << static_cast<int>(f)); }
	bool test(FSFlag f) const { return flags_ & (1 << static_cast<int>(f)); }
	void clearFlags() { flags_ = 0; }

	/**
	 * Move the entire frameset to another frameset object. This will
	 * invalidate the current frameset object as all memory buffers will be
	 * moved.
	 */
	void swapTo(ftl::data::FrameSet<FRAME> &);

    typedef FRAME Frame;
    typedef std::function<bool(ftl::data::FrameSet<FRAME> &)> Callback;

	/**
	 * Get the data from a data channel. This only works for the data channels
	 * and will throw an exception with any others.
	 */
	template <typename T> void get(ftl::codecs::Channel channel, T &params) const;

	/**
	 * Set the value of a channel. Some channels should not be modified via the
	 * non-const get method, for example the data channels.
	 */
	template <typename T> void create(ftl::codecs::Channel channel, const T &value);

	/**
	 * Access the raw data channel vector object.
	 */
	const std::vector<unsigned char> &getRawData(ftl::codecs::Channel c) const;

	/**
	 * Provide raw data for a data channel.
	 */
	void createRawData(ftl::codecs::Channel c, const std::vector<unsigned char> &v);

	/**
	 * Is there valid data in channel (either host or gpu). This does not
	 * verify that any memory or data exists for the channel.
	 */
	inline bool hasChannel(ftl::codecs::Channel channel) const {
		int c = static_cast<int>(channel);
		if (c == 66) return true;
		else if (c >= 2048) return data_channels_.has(channel);
		return false;
	}

	/**
	 * Check that a given frame is valid in this frameset.
	 */
	inline bool hasFrame(size_t ix) const { return (1 << ix) & mask; }

	/**
	 * Get the first valid frame in this frameset. No valid frames throws an
	 * exception.
	 */
	FRAME &firstFrame();

	const FRAME &firstFrame() const;

	void clearData() {
		data_.clear();
		data_channels_.clear();
	}

	ftl::codecs::Channels<2048> getDataChannels() const { return data_channels_; }

	private:
	std::unordered_map<int, std::vector<unsigned char>> data_;
	ftl::codecs::Channels<2048> data_channels_;
	std::atomic<int> flags_;
};

/**
 * Callback type for receiving video frames.
 */
//typedef std::function<bool(ftl::rgbd::FrameSet &)> VideoCallback;

/**
 * Abstract class for any generator of FrameSet structures. A generator
 * produces (decoded) frame sets at regular frame intervals depending on the
 * global timer settings. The `onFrameSet` callback may be triggered from any
 * thread and also may drop frames and not be called for a given timestamp.
 */
template <typename FRAMESET>
class Generator {
	public:
	Generator() {}
	virtual ~Generator() {}

	/** Number of frames in last frameset. This can change over time. */
	virtual size_t size()=0;

	/**
	 * Get the persistent state object for a frame. An exception is thrown
	 * for a bad index.
	 */
	virtual typename FRAMESET::Frame::State &state(size_t ix)=0;

	inline typename FRAMESET::Frame::State &operator[](int ix) { return state(ix); }

	/** Register a callback to receive new frame sets. */
	virtual void onFrameSet(const typename FRAMESET::Callback &)=0;
};

}
}

// === Implementations =========================================================

template <typename FRAME>
void ftl::data::FrameSet<FRAME>::swapTo(ftl::data::FrameSet<FRAME> &fs) {
	//UNIQUE_LOCK(fs.mtx, lk);
	std::unique_lock<std::shared_mutex> lk(fs.mtx);

	//if (fs.frames.size() != frames.size()) {
		// Assume "this" is correct and "fs" is not.
		fs.frames.resize(frames.size());
	//}

	fs.timestamp = timestamp;
	fs.count = static_cast<int>(count);
	fs.flags_ = (int)flags_;
	fs.mask = static_cast<unsigned int>(mask);
	fs.id = id;
	fs.pose = pose;

	for (size_t i=0; i<frames.size(); ++i) {
		frames[i].swapTo(ftl::codecs::Channels<0>::All(), fs.frames[i]);
	}

	std::swap(fs.data_, data_);
	fs.data_channels_ = data_channels_;
	data_channels_.clear();

	set(ftl::data::FSFlag::STALE);
}

// Default data channel implementation
template <typename FRAME>
// cppcheck-suppress *
template <typename T>
void ftl::data::FrameSet<FRAME>::get(ftl::codecs::Channel channel, T &params) const {
	if (static_cast<int>(channel) < static_cast<int>(ftl::codecs::Channel::Data)) throw FTL_Error("Cannot use generic type with non data channel");
	if (!hasChannel(channel)) throw FTL_Error("Data channel does not exist");

	const auto &i = data_.find(static_cast<int>(channel));
	if (i == data_.end()) throw FTL_Error("Data channel does not exist");

	auto unpacked = msgpack::unpack((const char*)(*i).second.data(), (*i).second.size());
	unpacked.get().convert(params);
}

template <typename FRAME>
// cppcheck-suppress *
template <typename T>
void ftl::data::FrameSet<FRAME>::create(ftl::codecs::Channel channel, const T &value) {
	if (static_cast<int>(channel) < static_cast<int>(ftl::codecs::Channel::Data)) throw FTL_Error("Cannot use generic type with non data channel");

	data_channels_ += channel;

	auto &v = *std::get<0>(data_.insert({static_cast<int>(channel),{}}));
	v.second.resize(0);
	ftl::util::FTLVectorBuffer buf(v.second);
	msgpack::pack(buf, value);
}

template <typename FRAME>
const std::vector<unsigned char> &ftl::data::FrameSet<FRAME>::getRawData(ftl::codecs::Channel channel) const {
	if (static_cast<int>(channel) < static_cast<int>(ftl::codecs::Channel::Data)) throw FTL_Error("Non data channel");
	if (!hasChannel(channel)) throw FTL_Error("Data channel does not exist");

	return data_.at(static_cast<int>(channel));
}

template <typename FRAME>
void ftl::data::FrameSet<FRAME>::createRawData(ftl::codecs::Channel c, const std::vector<unsigned char> &v) {
	data_.insert({static_cast<int>(c), v});
	data_channels_ += c;
}

template <typename FRAME>
FRAME &ftl::data::FrameSet<FRAME>::firstFrame() {
	for (size_t i=0; i<frames.size(); ++i) {
		if (hasFrame(i)) return frames[i];
	}
	throw FTL_Error("No frames in frameset");
}

template <typename FRAME>
const FRAME &ftl::data::FrameSet<FRAME>::firstFrame() const {
	for (size_t i=0; i<frames.size(); ++i) {
		if (hasFrame(i)) return frames[i];
	}
	throw FTL_Error("No frames in frameset");
}

#endif  // _FTL_DATA_FRAMESET_HPP_
