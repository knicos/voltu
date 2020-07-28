#ifndef _FTL_RENDER_COLLISIONS_HPP_
#define _FTL_RENDER_COLLISIONS_HPP_

#include <ftl/data/new_frameset.hpp>
#include <ftl/rgbd/frame.hpp>

namespace ftl {
namespace render {

void collision2touch(const ftl::rgbd::Frame &rgbdframe,
	const std::vector<float4> &collisions,
	const std::list<ftl::data::FrameSetPtr> &sets,
	uint32_t myid, float tmin, float tmax);

}
}

#endif