#ifndef _FTL_RGBD_SCREENCAPTURE_HPP_
#define _FTL_RGBD_SCREENCAPTURE_HPP_

#include <ftl/rgbd/detail/source.hpp>
#include <ftl/config.h>

namespace ftl {

namespace rgbd {

namespace detail {

#ifdef HAVE_X11
struct X11State;
typedef X11State ImplState;
#else
typedef int ImplState;
#endif

class ScreenCapture : public ftl::rgbd::detail::Source {
	public:
	explicit ScreenCapture(ftl::rgbd::Source *host);
	~ScreenCapture();

	bool capture(int64_t ts) { timestamp_ = ts; return true; };
	bool retrieve();
	bool compute(int n=-1, int b=-1);
	bool isReady();

	private:
	bool ready_;

	ImplState *impl_state_;
};

}
}
}

#endif  // _FTL_RGBD_SCREENCAPTURE_HPP_
