/*
 * Copyright 2019 Nicolas Pope
 */

#ifndef _FTL_DISPLAY_HPP_
#define _FTL_DISPLAY_HPP_

#include <ftl/config.h>
#include "../../../rgbd-sources/include/ftl/rgbd_source.hpp"

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"

#if defined HAVE_PCL
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#endif  // HAVE_PCL

namespace ftl {

/**
 * Multiple local display options for disparity or point clouds.
 */
class Display {
	private:
		std::string name_;
	public:
	enum style_t {
		STYLE_NORMAL, STYLE_DISPARITY, STYLE_DEPTH
	};

	public:
	explicit Display(nlohmann::json &config, std::string name);
	~Display();
	
	bool render(const cv::Mat &rgb, const cv::Mat &depth, const ftl::rgbd::CameraParameters &p);

#if defined HAVE_PCL
	bool render(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr);
#endif  // HAVE_PCL
	bool render(const cv::Mat &img, style_t s=STYLE_NORMAL);

	bool active() const;
	
	void wait(int ms);

	private:
	nlohmann::json config_;

#if defined HAVE_VIZ
	cv::viz::Viz3d *window_;
#endif  // HAVE_VIZ

#if defined HAVE_PCL
	pcl::visualization::PCLVisualizer::Ptr pclviz_;
#endif  // HAVE_PCL

	bool active_;
};
};

#endif  // _FTL_DISPLAY_HPP_
