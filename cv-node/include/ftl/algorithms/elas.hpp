/*
 * Copyright 2019 Nicolas Pope
 */

#ifndef _FTL_ALGORITHMS_ELAS_HPP_
#define _FTL_ALGORITHMS_ELAS_HPP_

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <elas.h>
#include <ftl/disparity.hpp>

namespace ftl {
namespace algorithms {

/**
 * LibELAS - Efficient Large-scale Stereo Matching 
 * @see http://www.cvlibs.net/software/libelas/
 */
class ELAS : public ftl::Disparity {
	public:
	explicit ELAS(nlohmann::json &config);

	void compute(const cv::Mat &l, const cv::Mat &r, cv::Mat &disp);

	/* Factory creator */
	static inline Disparity *create(nlohmann::json &config) {
		return new ELAS(config);
	}

	private:
	Elas::parameters param_;
	Elas *elas_;
};
};
};

#endif  // _FTL_ALGORITHMS_ELAS_HPP_

