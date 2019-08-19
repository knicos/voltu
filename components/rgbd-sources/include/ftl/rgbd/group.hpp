#ifndef _FTL_RGBD_GROUP_HPP_
#define _FTL_RGBD_GROUP_HPP_

#include <ftl/threads.hpp>
#include <ftl/timer.hpp>

#include <opencv2/opencv.hpp>
#include <vector>

namespace ftl {
namespace rgbd {

class Source;

/**
 * Represents a set of synchronised frames, each with two channels. This is
 * used to collect all frames from multiple computers that have the same
 * timestamp.
 */
struct FrameSet {
	int64_t timestamp;				// Millisecond timestamp of all frames
	std::vector<Source*> sources;	// All source objects involved.
	std::vector<cv::Mat> channel1;	// RGB
	std::vector<cv::Mat> channel2;	// Depth (usually)
	volatile int count;				// Number of valid frames
	volatile unsigned int mask;		// Mask of all sources that contributed
	bool stale;						// True if buffers have been invalidated
	MUTEX mtx;
};

// Allows a latency of 20 frames maximum
static const size_t kFrameBufferSize = 20;

/**
 * Manage a group of RGB-D sources to obtain synchronised sets of frames from
 * those sources. The Group class provides a synchronised callback mechanism
 * that uses the high precision timer to ensure that it is called once per
 * frame. The callback is not called if the frameset is not completed or
 * is unavailable for some other reason. By default if the required frame is
 * not available but there is an older frame available that has not been used
 * then it will be used. This can be disabled. It is also possible to allow
 * incomplete frames to be used, but this is disabled by default.
 */
class Group {
	public:
	Group();
	~Group();

	/**
	 * Add a new source to the group. Framesets generated prior to the source
	 * being added will still be valid and will not contain a frame from this
	 * source. Sets generated after addition will require a frame from this
	 * source.
	 */
	void addSource(ftl::rgbd::Source *);

	/**
	 * Add another group to this one. All sources in the other group are made
	 * available to this group in a synchronised way. There is additional
	 * overhead in supporting this as additional data copies are required
	 * internally for all the source frames.
	 */
	void addGroup(ftl::rgbd::Group *);

	/**
	 * Provide a function to be called once per frame with a valid frameset
	 * at the specified latency. The function may not be called under certain
	 * conditions (missing frameset). No guarantee is made about the timing
	 * accuracy of the call, it should be close to the frame point. This
	 * function may or may not block. It is intended that the buffers within
	 * the frameset are swapped during the function call, meaning that the
	 * frameset data is no longer valid upon returning.
	 */
	void sync(std::function<bool(FrameSet &)>);

	/** @deprecated */
	//bool getFrames(FrameSet &, bool complete=false);

	/** To be deprecated in favour of ftl::timer::setInterval.
	 */
	void setFPS(int fps);

	/**
	 * Set the minimum number of frames latency. The latency is from the most
	 * recent frame obtained, meaning that the timestamp of the input frames is
	 * the reference point, this may already be several frames old. Latency
	 * does not correspond to actual current time.
	 */
	void setLatency(int frames) { latency_ = frames; }

	void stop() {}

	private:
	std::vector<FrameSet> framesets_;
	std::vector<Source*> sources_;
	size_t head_;
	std::function<bool(FrameSet &)> callback_;
	MUTEX mutex_;
	int mspf_;
	int latency_;
	int64_t last_ts_;
	std::atomic<int> jobs_;
	volatile bool skip_;
	ftl::timer::TimerHandle cap_id_;
	ftl::timer::TimerHandle swap_id_;
	ftl::timer::TimerHandle main_id_;

	/* Insert a new frameset into the buffer, along with all intermediate
	 * framesets between the last in buffer and the new one.
	 */
	void _addFrameset(int64_t timestamp);

	void _retrieveJob(ftl::rgbd::Source *);
	void _computeJob(ftl::rgbd::Source *);

	/* Find a frameset with given latency in frames. */
	ftl::rgbd::FrameSet *_getFrameset(int f);
};

}
}

#endif  // _FTL_RGBD_GROUP_HPP_
