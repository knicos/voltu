#include <nanogui/layout.h>
#include <nanogui/label.h>
#include <nanogui/slider.h>

#include "soundctrl.hpp"
#include "../screen.hpp"

using ftl::gui2::PopupButton;
using ftl::gui2::VolumeButton;
using ftl::gui2::Screen;

VolumeButton::VolumeButton(nanogui::Widget *parent) :
	ftl::gui2::PopupButton(parent, "", ENTYPO_ICON_SOUND) {
	setChevronIcon(-1);

	muted_ = false;

	mPopup->setLayout(new nanogui::GroupLayout(15, 6, 14, 0));
	new nanogui::Label(mPopup, "Volume");
	slider_ = new nanogui::Slider(mPopup);

	slider_->setHighlightColor(dynamic_cast<Screen*>(screen())->getColor("highlight1"));
	slider_->setHeight(20);
	mPopup->setFixedWidth(200);

	slider_->setCallback([this](float value) {
		setValue(value);
		if (cb_) { cb_(value); }
	});
}

VolumeButton::~VolumeButton() {
}

void VolumeButton::setCallback(std::function<void(float)> cb) {
	cb_ = cb;
}

void VolumeButton::update() {
	slider_->setValue(value_);
	slider_->setHighlightedRange({0.0f, value_});

	if (muted_ || value_ == 0.0f) {
		setIcon(ICON_MUTED);
	}
	else if (value_ < 0.33){
		setIcon(ICON_VOLUME_1);
	}
	else if (value_ >= 0.67) {
		setIcon(ICON_VOLUME_3);
	}
	else {
		setIcon(ICON_VOLUME_2);
	}
}

void VolumeButton::setValue(float v) {
	value_ = v;
	setMuted(false);
	update();
}

float VolumeButton::value() {
	return muted_ ? 0.0f : value_;
}

void VolumeButton::setMuted(bool v) {
	if (muted_ == v) {
		return;
	}

	muted_ = v;
	if (muted_) {
		slider_->setHighlightColor(
			dynamic_cast<Screen*>(screen())->getColor("highlight1_disabled"));
	}
	else {
		slider_->setHighlightColor(
			dynamic_cast<Screen*>(screen())->getColor("highlight1"));
	}
	update();
}

bool VolumeButton::muted() {
	return muted_;
}

bool VolumeButton::mouseButtonEvent(const nanogui::Vector2i &p, int button, bool down, int modifiers) {
	parent()->setFocused(true);
	if (down && button == GLFW_MOUSE_BUTTON_2) {
		setMuted(!muted_);
		if (cb_) { cb_(value()); }
		return true;
	}
	else {
		return PopupButton::mouseButtonEvent(p, button, down, modifiers);
	}

}

bool VolumeButton::scrollEvent(const nanogui::Vector2i &p, const nanogui::Vector2f &rel) {
	setValue(std::min(std::max(0.0f, value_ + rel[1]*scroll_step_), 1.0f));
	if (cb_) { cb_(value()); }
	return true;
}
