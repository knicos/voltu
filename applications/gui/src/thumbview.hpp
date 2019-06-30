#ifndef _FTL_GUI_THUMBVIEW_HPP_
#define _FTL_GUI_THUMBVIEW_HPP_

#include <nanogui/imageview.h>

namespace ftl {
namespace gui {

class Screen;
class Camera;

class ThumbView : public nanogui::ImageView {
	public:
	ThumbView(nanogui::Widget *parent, ftl::gui::Screen *screen, ftl::gui::Camera *cam);
	~ThumbView();

	bool mouseButtonEvent(const nanogui::Vector2i &p, int button, bool down, int modifiers);

	void draw(NVGcontext *ctx);

	private:
	Screen *screen_;
	Camera *cam_;
};

}
}

#endif  // _FTL_GUI_THUMBVIEW_HPP_
