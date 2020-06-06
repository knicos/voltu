#ifndef _FTL_RGBD_IMAGE_HPP_
#define _FTL_RGBD_IMAGE_HPP_

namespace ftl {
namespace rgbd {
namespace detail {

class ImageSource : public ftl::rgbd::detail::Source {
	public:
	explicit ImageSource(ftl::rgbd::Source *host) : ftl::rgbd::detail::Source(host) {
	
	}
	ImageSource(ftl::rgbd::Source *host, const std::string &f) : ftl::rgbd::detail::Source(host) {

	}

	bool capture(int64_t ts) { timestamp_ = ts; return true; }
	bool retrieve() { return true; }
	bool compute(int64_t ts) { return false; };
	bool isReady() { return false; };
};

}
}
}

#endif  // _FTL_RGBD_IMAGE_HPP_
