#pragma once
#ifndef _FTL_RGBD_CAMERA_PARAMS_HPP_
#define _FTL_RGBD_CAMERA_PARAMS_HPP_

namespace ftl{
namespace rgbd {

struct CameraParameters {
	double fx;
	double fy;
	double cx;
	double cy;
	unsigned int width;
	unsigned int height;
	double minDepth;
	double maxDepth;
};

};
};

#endif
