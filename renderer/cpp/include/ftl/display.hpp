/*
 * Copyright 2019 Nicolas Pope
 */

#ifndef _FTL_DISPLAY_HPP_
#define _FTL_DISPLAY_HPP_

#include <ftl/config.h>

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"

namespace ftl {

/**
 * Multiple local display options for disparity or point clouds.
 */
class Display {
	public:
	explicit Display(nlohmann::json &config);
	~Display();
	
	void setCalibration(const cv::Mat &q) { q_ = q; }

	bool render(const cv::Mat &rgb, const cv::Mat &depth);

	bool active() const;
	
	void wait(int ms);

	private:
	cv::Mat q_;
	nlohmann::json config_;

	#if defined HAVE_VIZ
	cv::viz::Viz3d *window_;
	#endif  // HAVE_VIZ

	bool active_;
};
};

#endif  // _FTL_DISPLAY_HPP_

