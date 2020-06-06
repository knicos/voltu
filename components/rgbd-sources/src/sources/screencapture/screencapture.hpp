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
	void swap() override;
	bool retrieve();
	bool compute(int64_t ts);
	bool isReady();

	size_t getOffsetX() const { return (offset_x_ > full_width_-params_.width) ? full_width_-params_.width : offset_x_; }
	size_t getOffsetY() const { return (offset_y_ > full_height_-params_.height) ? full_height_-params_.height : offset_y_; }

	private:
	bool ready_;
	int64_t cap_ts_;
	int64_t cur_ts_;
	ftl::rgbd::Frame sframe_;

	size_t full_width_;
	size_t full_height_;
	size_t offset_x_;
	size_t offset_y_;

	ImplState *impl_state_;
};

}
}
}

#endif  // _FTL_RGBD_SCREENCAPTURE_HPP_
