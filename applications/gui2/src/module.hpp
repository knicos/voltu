/**
 * @file module.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Sebastian Hahta
 */

#pragma once

#include "view.hpp"
#include "inputoutput.hpp"

#include <ftl/configurable.hpp>
#include <nanogui/entypo.h>
#include <nanogui/button.h>

namespace ftl {
namespace gui2 {

class Screen;

class Module : public ftl::Configurable {
public:
	Module(nlohmann::json &config, Screen *screen, InputOutput *io) :
		Configurable(config), screen(screen), io(io) {}

	/** called by constructor */
	virtual void init() {};
	/** called before draw */
	virtual void update(double) {};
	virtual ~Module() {};

	ftl::gui2::Screen* const screen;

protected:
	ftl::gui2::InputOutput* const io;
};

}
}
