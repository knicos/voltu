#ifndef _FTL_RGBD_SCREENCAPTURE_HPP_
#define _FTL_RGBD_SCREENCAPTURE_HPP_

#include "../../basesource.hpp"
#include <ftl/config.h>
#include <ftl/codecs/touch.hpp>

namespace ftl {

namespace rgbd {

namespace detail {

#ifdef HAVE_X11
struct X11State;
typedef X11State ImplState;
#else
typedef int ImplState;
#endif

class ScreenCapture : public ftl::rgbd::BaseSourceImpl {
	public:
	explicit ScreenCapture(ftl::rgbd::Source *host);
	~ScreenCapture();

	bool capture(int64_t ts) override { return true; };
	bool retrieve(ftl::rgbd::Frame &frame) override;
	bool isReady() override;

	size_t getOffsetX() const { return (offset_x_ > full_width_-params_.width) ? full_width_-params_.width : offset_x_; }
	size_t getOffsetY() const { return (offset_y_ > full_height_-params_.height) ? full_height_-params_.height : offset_y_; }

	private:
	bool ready_;
	int64_t cap_ts_;
	int64_t cur_ts_;
	//ftl::rgbd::Frame sframe_;

	size_t full_width_;
	size_t full_height_;
	size_t offset_x_;
	size_t offset_y_;
	Eigen::Matrix4d pose_;
	bool do_update_params_ = false;
	bool pressed_ = false;
	ftl::codecs::Touch primary_touch_;

	ImplState *impl_state_;
	ftl::rgbd::Camera params_;
	//void _mouseClick(int button, int x, int y);

	void _singleTouch(const ftl::codecs::Touch &t);
	void _press();
	void _release();
	void _move(int x, int y);
	void _noTouch();
	void _multiTouch(const std::vector<ftl::codecs::Touch> &);
};

}
}
}

#endif  // _FTL_RGBD_SCREENCAPTURE_HPP_
