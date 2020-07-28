#pragma once

#include <nanogui/window.h>

namespace ftl {
namespace gui2 {
/**
 * Non-movable Window widget
 */
class FixedWindow : public nanogui::Window {
public:
	FixedWindow(nanogui::Widget *parent, const std::string name="") :
		nanogui::Window(parent, name) {};

	virtual bool mouseDragEvent(const nanogui::Vector2i&, const nanogui::Vector2i&, int, int) override { return false; }
	virtual ~FixedWindow() {}

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
}
