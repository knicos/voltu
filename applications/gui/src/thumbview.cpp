#include "thumbview.hpp"
#include "screen.hpp"
#include "camera.hpp"

using ftl::gui::ThumbView;
using ftl::gui::Screen;
using ftl::gui::Camera;

ThumbView::ThumbView(nanogui::Widget *parent, ftl::gui::Screen *screen, ftl::gui::Camera *cam)
 : ImageView(parent, 0), screen_(screen), cam_(cam) {
	 setCursor(nanogui::Cursor::Hand);
}

ThumbView::~ThumbView() {

}

bool ThumbView::mouseButtonEvent(const nanogui::Vector2i &p, int button, bool down, int modifiers) {
	if (button == 0 && !down) {
		screen_->setActiveCamera(cam_);
	}
}

void ThumbView::draw(NVGcontext *ctx) {
	ImageView::draw(ctx);

	nvgScissor(ctx, mPos.x(), mPos.y(), mSize.x(), mSize.y());
	nvgFontSize(ctx, 14);
	nvgFontFace(ctx, "sans-bold");
	nvgText(ctx, mPos.x() + 10, mPos.y()+mSize.y() - 10, cam_->source()->getURI().c_str(), NULL);
	nvgResetScissor(ctx);
}
