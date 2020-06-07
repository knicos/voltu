#pragma once
#ifndef _FTL_RGBD_MIDDLEBURY_SOURCE_HPP_
#define _FTL_RGBD_MIDDLEBURY_SOURCE_HPP_

#include <loguru.hpp>

#include <ftl/rgbd/source.hpp>
#include <ftl/cuda_common.hpp>

namespace ftl {
namespace rgbd {
namespace detail {

class Disparity;

class MiddleburySource : public detail::Source {
	public:
	explicit MiddleburySource(ftl::rgbd::Source *);
	MiddleburySource(ftl::rgbd::Source *, const std::string &dir);
	~MiddleburySource() {};

	bool capture(int64_t ts) { return true; }
	bool retrieve(ftl::rgbd::Frame &) { return true; }
	bool isReady() { return ready_; }

	private:
	bool ready_;
	Disparity *disp_;

	cv::cuda::Stream stream_;

	cv::cuda::GpuMat left_;
	cv::cuda::GpuMat right_;
	cv::cuda::GpuMat disp_tmp_;
	cv::cuda::GpuMat depth_tmp_;
	cv::Mat mask_l_;

	void _performDisparity();
};

}
}
}

#endif  // _FTL_RGBD_MIDDLEBURY_SOURCE_HPP_
