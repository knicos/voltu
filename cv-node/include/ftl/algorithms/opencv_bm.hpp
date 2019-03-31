/*
 * Copyright 2019 Nicolas Pope
 */

#ifndef _FTL_ALGORITHMS_OPENCV_BM_HPP_
#define _FTL_ALGORITHMS_OPENCV_BM_HPP_

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/ximgproc.hpp"
#include <opencv2/calib3d.hpp>
#include <ftl/disparity.hpp>

namespace ftl {
namespace algorithms {

/**
 * OpenCV Block Matching algorithm.
 */
class OpenCVBM : public ftl::Disparity {
	public:
	explicit OpenCVBM(nlohmann::json &config);
	
	void compute(const cv::Mat &l, const cv::Mat &r, cv::Mat &disp);

	static inline Disparity *create(nlohmann::json &config) {
		return new OpenCVBM(config);
	}
	
	private:
	cv::Ptr<cv::StereoBM> left_matcher_;
	cv::Ptr<cv::StereoMatcher> right_matcher_;
	cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter_;
};
};
};

#endif  // _FTL_ALGORITHMS_OPENCV_SGBM_HPP_

