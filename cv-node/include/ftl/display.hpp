/*
 * Copyright 2019 Nicolas Pope
 */

#ifndef _FTL_DISPLAY_HPP_
#define _FTL_DISPLAY_HPP_

#include <ftl/config.h>

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <ftl/calibrate.hpp>
#include "opencv2/highgui.hpp"

namespace ftl {

/**
 * Multiple local display options for disparity or point clouds.
 */
class Display {
	public:
	Display(const Calibrate &cal, nlohmann::json &config);
	~Display();

	bool render(const cv::Mat &rgb, const cv::Mat &depth);

	bool active() const;

	private:
	const ftl::Calibrate &calibrate_;
	nlohmann::json config_;

	#if defined HAVE_VIZ
	cv::viz::Viz3d *window_;
	#endif  // HAVE_VIZ

	bool active_;
};
};

#endif  // _FTL_DISPLAY_HPP_

