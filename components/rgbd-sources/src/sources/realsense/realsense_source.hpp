#pragma once
#ifndef _FTL_RGBD_REALSENSE_HPP_
#define _FTL_RGBD_REALSENSE_HPP_

#include "../../basesource.hpp"
#include <librealsense2/rs.hpp>
#include <string>

namespace ftl {

namespace rgbd {

namespace detail {

class RealsenseSource : public ftl::rgbd::BaseSourceImpl {
	public:
	explicit RealsenseSource(ftl::rgbd::Source *host);
	~RealsenseSource();

	bool capture(int64_t ts) override;
	bool retrieve(ftl::rgbd::Frame &frame) override;
	bool isReady() override;

	static bool supported();

	private:
	bool ready_;
	bool do_update_params_ = false;
    float scale_;
    rs2::pipeline pipe_;
    rs2::align align_to_depth_;
	rs2::frame rscolour_;
	ftl::rgbd::Camera params_;
	std::string name_;
	std::string serial_;
	cv::cuda::GpuMat tmp_depth_;
	cv::cuda::Stream stream_;
};

}
}
}

#endif  // _FTL_RGBD_REALSENSE_HPP_
