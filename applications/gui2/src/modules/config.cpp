/**
 * @file config.cpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Sebastian Hahta
 */

#include "config.hpp"

using ftl::gui2::ConfigCtrl;

void ConfigCtrl::init() {
	button = screen->addButton(ENTYPO_ICON_COG);
	button->setTooltip("Settings");
	button->setCallback([this](){
		button->setPushed(false);
		show();
	});
	button->setVisible(true);
}

void ConfigCtrl::show() {
	if (screen->childIndex(window) == -1) {
		window = new ftl::gui2::ConfigWindow(screen, io->master());
	}
	window->requestFocus();
	window->setVisible(true);
	screen->performLayout();
}

void ConfigCtrl::show(const std::string &uri) {
	ftl::gui2::ConfigWindow::buildForm(screen, uri);
}

ConfigCtrl::~ConfigCtrl() {
	
}
