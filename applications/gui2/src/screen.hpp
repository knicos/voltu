/**
 * @file screen.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Sebastian Hahta
 */

#pragma once

#include <nanogui/screen.h>
#include <nanogui/glutil.h>

#include <nanogui/toolbutton.h>

#include <map>
#include <memory>
#include <typeinfo>

#include "view.hpp"
#include "module.hpp"

namespace ftl {
namespace gui2 {

/**
 * FTL GUI main screen. Methods may only be called from main (GUI) threads
 * unless otherwise documented.
 */
class Screen : public nanogui::Screen {
public:
	explicit Screen();
	virtual ~Screen();

	virtual void drawAll() override;

	virtual bool keyboardEvent(int key, int scancode, int action, int modifiers) override;
	virtual bool keyboardCharacterEvent(unsigned int codepoint) override;

	void render(); // necessary?
	/** Redraw the screen (triggers an empty event). Thread safe. */
	void redraw();

	void activate(Module *ptr);

	/** set active view (existing object */
	void setView(ftl::gui2::View* view);
	/** set active view (create new object)*/
	template<typename T, typename ... Args>
	void setView(Args ... args);

	bool isActiveView(View* ptr) { return active_view_ == ptr; }

	/** Add a module.*/
	template<typename T, typename ... Args>
	T* addModule(const std::string &name, ftl::Configurable *config, Args ... args);

	/** Get a pointer to module. Module identified by name, exception thrown if not found */
	template<typename T>
	T* getModule(const std::string &name);

	/** Get a pointer to module. Module indentified by dynamic type from template parameter.
	 * Throws an exception if not found. If more than one possible match (same module
	 * loaded multiple times), return value can be any.
	 */
	template<typename T>
	T* getModule();

	template<typename T>
	T* getModuleNoExcept();

	// prever above template (explicit who manages delete)
	// template<typename T>
	// T* addModule(T* ptr) { return addModule_(ptr); }

	// TODO removeModule() as well?

	/** add a button to toolbar */
	template<typename T=nanogui::ToolButton, typename ... Args>
	T* addButton(Args ... args);

	/** themes/colors */
	nanogui::Theme* getTheme(const std::string &name);
	nanogui::Color getColor(const std::string &name);
	void setColor(const std::string &name, const nanogui::Color &c);

	// Implement in View or Screen? Add ID (address of creating instance)
	// to each error to prevent spam?
	/** Show error message popup */
	void showError(const std::string& title, const std::string &msg);

	nanogui::Vector2i viewSize(const nanogui::Vector2i &ws);
	nanogui::Vector2i viewSize();

private:
	Module* addModule_(const std::string &name, Module* ptr);

	//std::mutex mtx_; // not used: do not modify gui outside gui (main) thread
	std::map<std::string, ftl::gui2::Module*> modules_;
	std::map<std::string, nanogui::ref<nanogui::Theme>> themes_;
	std::map<std::string, nanogui::Color> colors_;

	nanogui::Widget *toolbar_;
	nanogui::Widget *tools_;

	ftl::gui2::View *active_view_;

	nanogui::MessageDialog* msgerror_;
	double last_draw_time_=0.0f;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename T, typename ... Args>
void Screen::setView(Args ... args) {
	setView(new T(this, args ...));
}

template<typename T, typename ... Args>
T* Screen::addModule(const std::string &name, ftl::Configurable *config, Args ... args) {
	static_assert(std::is_base_of<Module, T>::value);

	return dynamic_cast<T*>(
		addModule_(
			name,
			ftl::config::create<T>(config, name, args ...)
		)
	);
}

template<typename T>
T* Screen::getModule(const std::string &name) {
	static_assert(std::is_base_of<Module, T>::value);

	if (modules_.find(name) == modules_.end()) {
		throw ftl::exception("module: " + name + " not found");
	}

	auto* ptr = dynamic_cast<T*>(modules_[name]);

	if (ptr == nullptr) {
		throw ftl::exception("bad cast, module requested with wrong type");
	}

	return ptr;
}

template<typename T>
T* Screen::getModule() {
	static_assert(std::is_base_of<Module, T>::value);

	for (auto& [name, ptr] : modules_) {
		std::ignore = name;
		if (typeid(*ptr) == typeid(T)) {
			return dynamic_cast<T*>(ptr);
		}
	}

	throw ftl::exception("module not found");
}

template<typename T>
T* Screen::getModuleNoExcept() {
	static_assert(std::is_base_of<Module, T>::value);

	for (auto& [name, ptr] : modules_) {
		std::ignore = name;
		if (typeid(*ptr) == typeid(T)) {
			return dynamic_cast<T*>(ptr);
		}
	}

	return nullptr;
}

template<typename T, typename ... Args>
T* Screen::addButton(Args ... args) {
	static_assert(std::is_base_of<nanogui::Button, T>::value);

	T* button = new T(tools_, args ...);
	button->setIconExtraScale(1.5f);
	button->setTheme(themes_["toolbutton"]);
	button->setFixedSize(nanogui::Vector2i(40, 40));
	performLayout();
	return button;
}

}
}
