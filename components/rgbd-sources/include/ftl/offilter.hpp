#pragma once

#include <ftl/config.h>
#include <ftl/rgbd/frame.hpp>

#ifdef HAVE_OPTFLOW
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaoptflow.hpp>

namespace ftl {
namespace rgbd {

class OFDisparityFilter {
public:
	OFDisparityFilter() : n_max_(0), threshold_(0.0) {}
	OFDisparityFilter(cv::Size size, int n_frames, float threshold);
	void filter(ftl::rgbd::Frame &frame, cv::cuda::Stream &stream);

private:
	int n_max_;
	float threshold_;

	cv::cuda::GpuMat disp_old_;
};

}
}

#endif  // HAVE_OPTFLOW
