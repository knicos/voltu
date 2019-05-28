/*
 * Copyright 2019 Nicolas Pope
 */

#ifndef _FTL_ALGORITHMS_FIXSTARS_SGM_HPP_
#define _FTL_ALGORITHMS_FIXSTARS_SGM_HPP_

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <libsgm.h>
#include "../disparity.hpp"
#include <opencv2/cudastereo.hpp>

namespace ftl {
namespace algorithms {

/**
 * Fixstars libSGM stereo matcher. 
 * @see https://github.com/fixstars/libSGM
 *
 * NOTE: We are using a modified version that supports disparity of 256.
 * @see https://github.com/knicos/libSGM
 */
class FixstarsSGM : public ftl::Disparity {
	public:
	explicit FixstarsSGM(nlohmann::json &config);

	void compute(const cv::Mat &l, const cv::Mat &r, cv::Mat &disp) override;
	void compute(const cv::Mat &l, const cv::Mat &r, cv::Mat &disp, const cv::Mat &mask_l) override;
	
	/* Factory creator */
	static inline Disparity *create(nlohmann::json &config) {
		return new FixstarsSGM(config);
	}

	private:
	bool use_filter_;
	cv::Ptr<cv::cuda::DisparityBilateralFilter> filter_;
	sgm::StereoSGM *ssgm_;
};
};
};

#endif  // _FTL_ALGORITHMS_FIXSTARS_SGM_HPP_

