/*
 * Copyright 2019 Nicolas Pope
 */

#ifndef _FTL_ALGORITHMS_OPENCV_SGBM_HPP_
#define _FTL_ALGORITHMS_OPENCV_SGBM_HPP_

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/ximgproc.hpp"
#include <opencv2/calib3d.hpp>
#include "../disparity.hpp"
#include <ftl/configuration.hpp>

namespace ftl {
namespace algorithms {

/**
 * OpenCV Semi Global Matching algorithm.
 */
class OpenCVSGBM : public ftl::Disparity {
	public:
	explicit OpenCVSGBM(nlohmann::json &config);
	
	void compute(const cv::Mat &l, const cv::Mat &r, cv::Mat &disp);

	static inline Disparity *create(ftl::Configurable *p, const std::string &name) {
		return ftl::create<OpenCVSGBM>(p, name);
	}
	
	private:
	cv::Ptr<cv::StereoSGBM> left_matcher_;
	cv::Ptr<cv::StereoMatcher> right_matcher_;
	cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter_;
};
};
};

#endif  // _FTL_ALGORITHMS_OPENCV_SGBM_HPP_

