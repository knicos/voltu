#pragma once
#ifndef _FTL_RGBD_CAMERA_PARAMS_HPP_
#define _FTL_RGBD_CAMERA_PARAMS_HPP_

namespace ftl{
namespace rgbd {

struct Camera {
	double fx;
	double fy;
	double cx;
	double cy;
	unsigned int width;
	unsigned int height;
	double minDepth;
	double maxDepth;
	double baseline;
};

};
};

#endif
