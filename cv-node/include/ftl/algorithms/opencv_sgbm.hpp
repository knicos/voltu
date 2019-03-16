#ifndef _FTL_ALGORITHMS_OPENCV_SGBM_HPP_
#define _FTL_ALGORITHMS_OPENCV_SGBM_HPP_

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/ximgproc.hpp"
#include <opencv2/calib3d.hpp>
#include <ftl/disparity.hpp>

namespace ftl {
namespace algorithms {
class OpenCVSGBM : public ftl::Disparity {
	public:
	OpenCVSGBM();
	
	void compute(const cv::Mat &l, const cv::Mat &r, cv::Mat &disp);

	static inline Disparity *create() { return new OpenCVSGBM(); }
	
	private:
	cv::Ptr<cv::StereoSGBM> left_matcher_;
	cv::Ptr<cv::StereoMatcher> right_matcher_;
	cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter_;
};
};
};

#endif // _FTL_ALGORITHMS_OPENCV_SGBM_HPP_

