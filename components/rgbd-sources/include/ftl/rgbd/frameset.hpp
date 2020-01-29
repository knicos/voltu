#ifndef _FTL_RGBD_FRAMESET_HPP_
#define _FTL_RGBD_FRAMESET_HPP_

#include <ftl/threads.hpp>
#include <ftl/timer.hpp>
#include <ftl/rgbd/frame.hpp>

#include <opencv2/opencv.hpp>
#include <vector>

namespace ftl {
namespace rgbd {

// Allows a latency of 20 frames maximum
static const size_t kMaxFramesets = 15;
static const size_t kMaxFramesInSet = 32;

class Source;

/**
 * Represents a set of synchronised frames, each with two channels. This is
 * used to collect all frames from multiple computers that have the same
 * timestamp.
 */
struct FrameSet {
	int id=0;
	int64_t timestamp;				// Millisecond timestamp of all frames
	std::vector<ftl::rgbd::Frame> frames;
	std::atomic<int> count;				// Number of valid frames
	std::atomic<unsigned int> mask;		// Mask of all sources that contributed
	bool stale;						// True if buffers have been invalidated
	SHARED_MUTEX mtx;

	/**
	 * Upload all specified host memory channels to GPU memory.
	 */
	void upload(ftl::codecs::Channels<0>, cudaStream_t stream=0);

	/**
	 * Download all specified GPU memory channels to host memory.
	 */
	void download(ftl::codecs::Channels<0>, cudaStream_t stream=0);

	/**
	 * Move the entire frameset to another frameset object. This will
	 * invalidate the current frameset object as all memory buffers will be
	 * moved.
	 */
	void swapTo(ftl::rgbd::FrameSet &);

	/**
	 * Clear all channels and all memory allocations within those channels.
	 * This will perform a resetFull on all frames in the frameset.
	 */
	void resetFull();
};

/**
 * Callback type for receiving video frames.
 */
typedef std::function<bool(ftl::rgbd::FrameSet &)> VideoCallback;

/**
 * Abstract class for any generator of FrameSet structures. A generator
 * produces (decoded) frame sets at regular frame intervals depending on the
 * global timer settings. The `onFrameSet` callback may be triggered from any
 * thread and also may drop frames and not be called for a given timestamp.
 */
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
	virtual ftl::rgbd::FrameState &state(size_t ix)=0;

	inline ftl::rgbd::FrameState &operator[](int ix) { return state(ix); }

	/** Register a callback to receive new frame sets. */
	virtual void onFrameSet(const ftl::rgbd::VideoCallback &)=0;
};

/**
 * Accept frames and generate framesets as they become completed. This can
 * directly act as a generator of framesets, each frameset being generated
 * by the global timer. Once the expected number of frames have been received,
 * a frameset is marked as complete and can then be passed to the callback at
 * the appropriate time. If frames are generated faster than consumed then they
 * are buffered and merged into a single frameset. The buffer has a limited size
 * so a longer delay in a callback will cause buffer failures. If frames are
 * generated below framerate then the on frameset callback is just not called.
 */
class Builder : public Generator {
	public:
	Builder();
	~Builder();

	size_t size() override;

	ftl::rgbd::FrameState &state(size_t ix) override;

	void onFrameSet(const ftl::rgbd::VideoCallback &) override;

	/**
	 * Add a new frame at a given timestamp.
	 */
	//void push(int64_t timestamp, size_t ix, ftl::rgbd::Frame &f);

	/**
	 * Instead of pushing a frame, find or create a direct reference to one.
	 */
	ftl::rgbd::Frame &get(int64_t timestamp, size_t ix);

	/**
	 * Mark a frame as completed.
	 */
	void completed(int64_t ts, size_t ix);

	void setName(const std::string &name);

	private:
	std::list<FrameSet*> framesets_;  // Active framesets
	std::list<FrameSet*> allocated_;  // Keep memory allocations

	size_t head_;
	ftl::rgbd::VideoCallback callback_;
	MUTEX mutex_;
	int mspf_;
	float latency_;
	float fps_;
	int stats_count_;
	int64_t last_ts_;
	std::atomic<int> jobs_;
	volatile bool skip_;
	ftl::timer::TimerHandle main_id_;
	size_t size_;
	std::vector<ftl::rgbd::FrameState*> states_;

	std::string name_;

	/* Insert a new frameset into the buffer, along with all intermediate
	 * framesets between the last in buffer and the new one.
	 */
	ftl::rgbd::FrameSet *_addFrameset(int64_t timestamp);

	/* Find a frameset with given latency in frames. */
	ftl::rgbd::FrameSet *_getFrameset();

	/* Search for a matching frameset. */
	ftl::rgbd::FrameSet *_findFrameset(int64_t ts);
	void _freeFrameset(ftl::rgbd::FrameSet *);

	void _recordStats(float fps, float latency);
};

}
}

#endif  // _FTL_RGBD_FRAMESET_HPP_
