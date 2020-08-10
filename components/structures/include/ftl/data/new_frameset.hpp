#ifndef _FTL_DATA_NFRAMESET_HPP_
#define _FTL_DATA_NFRAMESET_HPP_

#include <ftl/threads.hpp>
#include <ftl/timer.hpp>
#include <ftl/data/new_frame.hpp>
#include <ftl/utility/intrinsics.hpp>
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
	PARTIAL = 1,
	DISCARD = 4,
	AUTO_SEND = 8
};

/**
 * Represents a set of synchronised frames, each with two channels. This is
 * used to collect all frames from multiple computers that have the same
 * timestamp.
 */
class FrameSet : public ftl::data::Frame {
	private:
	//FrameSet(Pool *ppool, Session *parent, uint32_t pid, int64_t ts) :
	//	Frame(ppool, parent, pid | 0xFF, ts) {};


	public:
	FrameSet(Pool *ppool, FrameID pid, int64_t ts, size_t psize=1);
	~FrameSet();

	//int id=0;
	//int64_t timestamp;				// Millisecond timestamp of all frames
	int64_t localTimestamp;
	std::vector<Frame> frames;
	//std::atomic<int> count=0;				// Actual packet count
	//std::atomic<int> expected=0;				// Expected packet count
	std::atomic<unsigned int> mask;		// Mask of all sources that contributed
	//std::atomic<int> flush_count;		// How many channels have been flushed
	SHARED_MUTEX smtx;

	//Eigen::Matrix4d pose;  // Set to identity by default.

	inline void set(FSFlag f) { flags_ |= (1 << static_cast<int>(f)); }
	inline void clear(FSFlag f) { flags_ &= ~(1 << static_cast<int>(f)); }
	inline bool test(FSFlag f) const { return flags_ & (1 << static_cast<int>(f)); }
	inline void clearFlags() { flags_ = 0; }

	std::unordered_set<ftl::codecs::Channel> channels();

	/**
	 * Move the entire frameset to another frameset object. This will
	 * invalidate the current frameset object as all memory buffers will be
	 * moved.
	 */
	void moveTo(ftl::data::FrameSet &);

	/**
	 * Mark a frame as being completed. This modifies the mask and count
	 * members.
	 */
	void completed(size_t ix);

	inline void markPartial() {
		set(ftl::data::FSFlag::PARTIAL);
	}

	/**
	 * Are all frames complete within this frameset?
	 */
	inline bool isComplete() { return mask != 0 && ftl::popcount(mask) == frames.size(); }

	/**
	 * Check that a given frame is valid in this frameset.
	 */
	inline bool hasFrame(size_t ix) const { return (1 << ix) & mask; }

	/**
	 * Get the first valid frame in this frameset. No valid frames throws an
	 * exception.
	 */
	Frame &firstFrame();

	const Frame &firstFrame() const;

	inline Frame &operator[](int ix) { return frames[ix]; }
	inline const Frame &operator[](int ix) const { return frames[ix]; }

	/**
	 * Flush all frames in the frameset.
	 */
	void flush();

	/**
	 * Store all frames.
	 */
	void store();

	/**
	 * Flush a channel for all frames in the frameset.
	 */
	void flush(ftl::codecs::Channel);

	void resize(size_t s);

	/**
	 * Force a change to all frame timestamps. This is generally used internally
	 * to allow frameset buffering in advance of knowing an exact timestamp.
	 * The method will update the timestamps of all contained frames and the
	 * frameset itself.
	 */
	void changeTimestamp(int64_t ts);

	/**
	 * Make a frameset from a single frame. It borrows the pool, id and
	 * timestamp from the frame and creates a wrapping frameset instance.
	 */
	static std::shared_ptr<FrameSet>  fromFrame(Frame &);

	/**
	 * Check if channel has changed in any frames.
	 */
	bool hasAnyChanged(ftl::codecs::Channel) const;

	private:
	std::atomic<int> flags_;
};

using FrameSetPtr = std::shared_ptr<ftl::data::FrameSet>;
using FrameSetCallback = std::function<bool(const FrameSetPtr&)>;

class Generator {
	public:
	virtual ftl::Handle onFrameSet(const FrameSetCallback &)=0;
};

/**
 * Callback type for receiving video frames.
 */
//typedef std::function<bool(ftl::rgbd::FrameSet &)> VideoCallback;

}
}

#endif  // _FTL_DATA_FRAMESET_HPP_
