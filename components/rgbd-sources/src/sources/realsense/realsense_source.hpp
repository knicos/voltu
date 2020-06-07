#pragma once
#ifndef _FTL_RGBD_REALSENSE_HPP_
#define _FTL_RGBD_REALSENSE_HPP_

#include <ftl/rgbd/detail/source.hpp>
#include <librealsense2/rs.hpp>
#include <string>

namespace ftl {

namespace rgbd {

namespace detail {

class RealsenseSource : public ftl::rgbd::BaseSourceImpl {
	public:
	explicit RealsenseSource(ftl::rgbd::Source *host);
	~RealsenseSource();

	bool capture(int64_t ts) { return true; }
	bool retrieve(ftl::rgbd::Frame &frame);
	bool isReady();

	private:
	bool ready_;
    float scale_;
    rs2::pipeline pipe_;
    rs2::align align_to_depth_;
	rs2::frame rscolour_;
};

}
}
}

#endif  // _FTL_RGBD_REALSENSE_HPP_
