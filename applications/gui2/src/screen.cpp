/**
 * @file screen.cpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Sebastian Hahta
 */

#include <nanogui/opengl.h>
#include <nanogui/glutil.h>
#include <nanogui/screen.h>
#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <nanogui/imageview.h>
#include <nanogui/label.h>
#include <nanogui/toolbutton.h>
#include <nanogui/popupbutton.h>

#include <Eigen/Eigen>

#include "screen.hpp"
#include "widgets/window.hpp"

#include <nanogui/messagedialog.h>

#include <loguru.hpp>

using std::min;
using std::max;

using Eigen::Vector2i;

using ftl::gui2::Screen;

static const int toolbar_w = 50;
static const Vector2i wsize(1280,720);

Screen::Screen() :
		nanogui::Screen(wsize, "FT-Lab Remote Presence"),
		toolbar_(nullptr),
		active_view_(nullptr), msgerror_(nullptr) {

	using namespace nanogui;

	setSize(wsize);

	toolbar_ = new FixedWindow(this);
	toolbar_->setPosition({0, 0});
	toolbar_->setWidth(toolbar_w);
	toolbar_->setHeight(height());
	toolbar_->setTheme(getTheme("media"));

	setResizeCallback([this](const Vector2i &s) {
		toolbar_->setFixedSize({toolbar_->width(), s[1]});
		toolbar_->setPosition({0, 0});
		if (active_view_) {
			active_view_->setSize(viewSize(s));
		}
		performLayout();
	});

	tools_ = new Widget(toolbar_);
	tools_->setLayout(new BoxLayout( Orientation::Vertical,
									Alignment::Middle, 0, 10));
	tools_->setPosition(Vector2i(5,10));

	setVisible(true);
	performLayout();
}

Screen::~Screen() {
	 // removes view; onClose() callback can depend on module
	if (active_view_) {
		this->removeChild(active_view_);
		active_view_ = nullptr;
	}

	for (auto [name, ptr] : modules_) {
		std::ignore = name;
		delete ptr;
	}
}


nanogui::Theme* Screen::getTheme(const std::string &name) {
	if (themes_.count(name) == 0) {
		themes_[name] = new nanogui::Theme(*theme());
	}
	return themes_[name];
}

nanogui::Color Screen::getColor(const std::string &name) {
	if (colors_.count(name) == 0) {
		return nanogui::Color(0, 0, 0, 0);
	}
	return colors_[name];
}

void Screen::setColor(const std::string &name, const nanogui::Color &c) {
	colors_[name] = c;
}

void Screen::redraw() {
	// glfwPostEmptyEvent() is safe to call from any thread
	// https://www.glfw.org/docs/3.3/intro_guide.html#thread_safety
	glfwPostEmptyEvent();
}

nanogui::Vector2i Screen::viewSize(const nanogui::Vector2i &ws) {
	return {ws.x(), ws.y()};
}

nanogui::Vector2i Screen::viewSize() {
	return viewSize(size());
}


void Screen::showError(const std::string&title, const std::string& msg) {
	// FIXME: This isn't thread safe?
	if (msgerror_) { return; }
	msgerror_ = new nanogui::MessageDialog
		(screen(), nanogui::MessageDialog::Type::Warning, title, msg);
	msgerror_->setModal(false);
	msgerror_->setCallback([this](int){
		msgerror_ = nullptr;
	});
}

void Screen::setView(ftl::gui2::View *view) {

	view->setPosition({0, 0});

	view->setTheme(getTheme("view"));
	view->setVisible(true);

	if (childIndex(view) == -1) {
		addChild(view);
	}

	if (active_view_) {
		active_view_->setVisible(false);

		// View requires same cleanup as Window (see screen.cpp) before removed.
		if (std::find(mFocusPath.begin(), mFocusPath.end(), active_view_) != mFocusPath.end()) {
			mFocusPath.clear();
		}
		if (mDragWidget == active_view_) {
			mDragWidget = nullptr;
		}

		removeChild(active_view_);
	}

	// all windows should be in front of new view
	mChildren.erase(std::remove(mChildren.begin(), mChildren.end(), view), mChildren.end());
	mChildren.insert(mChildren.begin(), view);

	active_view_ = view;
	LOG(INFO) << "number of children (Screen): "<< mChildren.size();

	// hide all popups (TODO: only works on toolbar at the moment)
	for (nanogui::Widget* widget : tools_->children()) {
		if (auto button = dynamic_cast<nanogui::PopupButton*>(widget)) {
			button->setPushed(false);
		}
	}

	performLayout();
}

void Screen::render() {
	if (active_view_) {
		active_view_->render();
	}
}

ftl::gui2::Module* Screen::addModule_(const std::string &name, ftl::gui2::Module* ptr) {
	ptr->init();
	if (modules_.find(name) != modules_.end()) {
		LOG(WARNING) << "Module " << name  << " already loaded. Removing old module";
		delete modules_[name];
	}

	modules_[name] = ptr;
	return ptr;
}


bool Screen::keyboardEvent(int key, int scancode, int action, int modifiers) {

	if (nanogui::Screen::keyboardEvent(key, scancode, action, modifiers)) {
		return true;
	}

	if (active_view_) {
		// event not processed in any focused widget
		return active_view_->keyboardEvent(key, scancode, action, modifiers);
	}

	return false;
}

bool Screen::keyboardCharacterEvent(unsigned int codepoint) {

	if (nanogui::Screen::keyboardCharacterEvent(codepoint)) {
		return true;
	}

	if (active_view_) {
		// event not processed in any focused widget
		return active_view_->keyboardCharacterEvent(codepoint);
	}

	return false;
}

void Screen::drawAll() {
	double now = glfwGetTime();
	double delta = now - last_draw_time_;
	for (const auto& [name, mod] : modules_) {
		mod->update(delta);
	}
	last_draw_time_ = now;
	nanogui::Screen::drawAll();
}
