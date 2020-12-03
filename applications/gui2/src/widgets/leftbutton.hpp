/**
 * @file leftbutton.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Sebastian Hahta
 */

#pragma once
#include <nanogui/button.h>

namespace ftl {
namespace gui2 {

/**
 * Allow left aligned button text.
 */
class LeftButton : public nanogui::Button {
public:
	LeftButton(nanogui::Widget *parent, const std::string &caption = "",
				int buttonIcon = 0) : nanogui::Button(parent, caption, buttonIcon) {};
	virtual ~LeftButton() {};

	virtual void draw(NVGcontext* ctx) override;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
}
