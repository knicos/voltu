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

/**
 * Represents a set of synchronised frames, each with two channels. This is
 * used to collect all frames from multiple computers that have the same
 * timestamp.
 */
template <typename FRAME>
struct FrameSet {
	int id=0;
	int64_t timestamp;				// Millisecond timestamp of all frames
	std::vector<FRAME> frames;
	std::atomic<int> count;				// Number of valid frames
	std::atomic<unsigned int> mask;		// Mask of all sources that contributed
	bool stale;						// True if buffers have been invalidated
	SHARED_MUTEX mtx;

	/**
	 * Upload all specified host memory channels to GPU memory.
	 */
	//void upload(ftl::codecs::Channels<0>, cudaStream_t stream=0);

	/**
	 * Download all specified GPU memory channels to host memory.
	 */
	//void download(ftl::codecs::Channels<0>, cudaStream_t stream=0);

	/**
	 * Move the entire frameset to another frameset object. This will
	 * invalidate the current frameset object as all memory buffers will be
	 * moved.
	 */
	void swapTo(ftl::data::FrameSet<FRAME> &);

	/**
	 * Clear all channels and all memory allocations within those channels.
	 * This will perform a resetFull on all frames in the frameset.
	 */
	//void resetFull();

    typedef FRAME Frame;
    typedef std::function<bool(ftl::data::FrameSet<FRAME> &)> Callback;
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

#endif  // _FTL_DATA_FRAMESET_HPP_
