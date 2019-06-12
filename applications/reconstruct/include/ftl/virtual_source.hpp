#pragma once
#ifndef _FTL_RGBD_VIRTUAL_HPP_
#define _FTL_RGBD_VIRTUAL_HPP_

#include <ftl/rgbd/source.hpp>

class CUDARayCastSDF;

namespace ftl {
namespace voxhash {
	class SceneRep;
}

namespace rgbd {

/**
 * RGBD source from either a stereo video file with left + right images, or
 * direct from two camera devices. A variety of algorithms are included for
 * calculating disparity, before converting to depth.  Calibration of the images
 * is also performed.
 */
class VirtualSource : public ftl::rgbd::detail::Source {
	public:
	VirtualSource(ftl::rgbd::Source*);
	~VirtualSource();

	void setScene(ftl::voxhash::SceneRep *);

	bool grab();
	//void getRGBD(cv::Mat &rgb, cv::Mat &depth);
	bool isReady();

	private:
	ftl::voxhash::SceneRep *scene_;
	CUDARayCastSDF *rays_;
	bool ready_;
	cv::Mat idepth_;
};

}
}

#endif  // _FTL_RGBD_VIRTUAL_HPP_