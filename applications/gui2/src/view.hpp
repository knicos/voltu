/**
 * @file view.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Sebastian Hahta
 */

#pragma once

#include <nanogui/widget.h>
#include "inputoutput.hpp"

namespace ftl {
namespace gui2 {

class Screen;

class View : public nanogui::Widget {
public:
	View(Screen* parent);

	virtual ~View() {
		if(cb_close_) {
			cb_close_();
		}
	}

	/** onClose callback; view closed (destroyed) */
	void onClose(const std::function<void()> &cb) { cb_close_ = cb; }

	virtual void render() {}// TODO remove if VR works?

	inline Screen *gui() const { return screen_; }

private:
	std::function<void()> cb_close_;
	Screen *screen_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

};
};
