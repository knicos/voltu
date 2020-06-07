#pragma once
#ifndef _FTL_RGBD_PYLON_HPP_
#define _FTL_RGBD_PYLON_HPP_

#include <ftl/rgbd/detail/source.hpp>
#include <string>

namespace Pylon {
class CBaslerUniversalInstantCamera;
}

namespace ftl {

namespace rgbd {

namespace detail {

class PylonSource : public ftl::rgbd::BaseSourceImpl {
	public:
	explicit PylonSource(ftl::rgbd::Source *host);
	~PylonSource();

	bool capture(int64_t ts);
	bool retrieve(ftl::rgbd::Frame &frame);
	bool isReady();

	private:
	bool ready_;
	Pylon::CBaslerUniversalInstantCamera *lcam_;
	Pylon::CBaslerUniversalInstantCamera *rcam_;
	cv::Mat tmp_;

	void _configureCamera(Pylon::CBaslerUniversalInstantCamera *cam);
};

}
}
}

#endif  // _FTL_RGBD_PYLON_HPP_
