#ifndef _FTL_RGBD_FRAMESET_HPP_
#define _FTL_RGBD_FRAMESET_HPP_

#include <ftl/rgbd/frame.hpp>

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
	std::vector<ftl::rgbd::Frame> frames;
	std::atomic<int> count;				// Number of valid frames
	std::atomic<unsigned int> mask;		// Mask of all sources that contributed
	bool stale;						// True if buffers have been invalidated
	SHARED_MUTEX mtx;
};

}
}

#endif  // _FTL_RGBD_FRAMESET_HPP_
