#ifndef _FTL_RGBD_FRAMESET_HPP_
#define _FTL_RGBD_FRAMESET_HPP_

#include <ftl/threads.hpp>
#include <ftl/timer.hpp>
#include <ftl/rgbd/frame.hpp>
#include <ftl/data/new_frameset.hpp>

//#include <opencv2/core.hpp>
#include <vector>

namespace ftl {
namespace rgbd {

// Allows a latency of 20 frames maximum
static const size_t kMaxFramesets = 15;
static const size_t kMaxFramesInSet = 32;

typedef ftl::data::FrameSet FrameSet;

}
}

#endif  // _FTL_RGBD_FRAMESET_HPP_
