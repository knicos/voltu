#include <nanogui/widget.h>

#include "view.hpp"
#include "screen.hpp"

using ftl::gui2::View;

View::View(Screen* screen) : nanogui::Widget(screen), screen_(screen) {
	setSize(screen_->viewSize());
}
