#pragma once

#include <ftl/config.h>

#ifdef HAVE_OPTFLOW
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaoptflow.hpp>

namespace ftl {
namespace rgbd {

class OFDisparityFilter {
public:
	OFDisparityFilter() : n_max_(0), threshold_(0.0), size_(0, 0) {} // TODO: invalid state
	OFDisparityFilter(cv::Size size, int n_frames, float threshold);
	void filter(cv::Mat &disp, const cv::Mat &rgb);

private:
	int n_;
	int n_max_;
	float threshold_;
	cv::Size size_;

	cv::Mat disp_;
	cv::Mat gray_;

	cv::Mat flowxy_;
	cv::Mat flowxy_up_;

	cv::Ptr<cv::cuda::NvidiaOpticalFlow_1_0> nvof_;
};

}
}

#endif  // HAVE_OPTFLOW
