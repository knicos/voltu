#ifndef _FTL_RGBD_IMAGE_HPP_
#define _FTL_RGBD_IMAGE_HPP_

#include "../../basesource.hpp"

namespace ftl {
namespace rgbd {
namespace detail {

class ImageSource : public ftl::rgbd::BaseSourceImpl {
	public:
	explicit ImageSource(ftl::rgbd::Source *host) : ftl::rgbd::BaseSourceImpl(host) {
	
	}
	ImageSource(ftl::rgbd::Source *host, const std::string &f) : ftl::rgbd::BaseSourceImpl(host) {

	}

	bool capture(int64_t ts) { return true; }
	bool retrieve(ftl::rgbd::Frame &) { return true; }
	bool isReady() { return false; };
};

}
}
}

#endif  // _FTL_RGBD_IMAGE_HPP_
