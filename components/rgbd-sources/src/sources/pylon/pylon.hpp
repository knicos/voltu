#pragma once
#ifndef _FTL_RGBD_PYLON_HPP_
#define _FTL_RGBD_PYLON_HPP_

#include <ftl/rgbd/detail/source.hpp>
#include <string>

namespace Pylon {
class CInstantCamera;
}

namespace ftl {

namespace rgbd {

namespace detail {

class PylonSource : public ftl::rgbd::detail::Source {
	public:
	explicit PylonSource(ftl::rgbd::Source *host);
	~PylonSource();

	void swap();
	bool capture(int64_t ts);
	bool retrieve();
	bool compute(int n=-1, int b=-1);
	bool isReady();

	private:
	bool ready_;
	Pylon::CInstantCamera *lcam_;
	Frame frames_[2];
};

}
}
}

#endif  // _FTL_RGBD_PYLON_HPP_
