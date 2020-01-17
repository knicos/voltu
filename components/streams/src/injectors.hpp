#ifndef _FTL_STREAM_INJECTORS_HPP_
#define _FTL_STREAM_INJECTORS_HPP_

#include <ftl/streams/stream.hpp>

namespace ftl {
namespace stream {

void injectPose(ftl::stream::Stream *stream, const ftl::rgbd::FrameSet &fs, int ix);
void injectCalibration(ftl::stream::Stream *stream, const ftl::rgbd::FrameSet &fs, int ix, bool right=false);
void injectConfig(ftl::stream::Stream *stream, const ftl::rgbd::FrameSet &fs, int ix);

void injectPose(ftl::stream::Stream *stream, const ftl::rgbd::Frame &fs, int64_t ts, int ix);
void injectCalibration(ftl::stream::Stream *stream, const ftl::rgbd::Frame &fs, int64_t ts, int ix, bool right=false);

}
}

#endif  // _FTL_STREAM_INJECTORS_HPP_
