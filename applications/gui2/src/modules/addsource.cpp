/**
 * @file addsource.cpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#include "addsource.hpp"

using ftl::gui2::AddCtrl;

void AddCtrl::init() {
	button = screen->addButton(ENTYPO_ICON_PLUS);
	button->setTooltip("Add New Source");
	button->setCallback([this](){
		button->setPushed(false);
		show();
	});
	button->setVisible(true);
}

void AddCtrl::show() {
	// Note: By chance, the pointer can in fact pass this test as another
	// widget gets allocated to the exact same address
	if (!window || screen->childIndex(window) == -1) {
		window = new ftl::gui2::AddSourceWindow(screen, this);
	}
	window->setVisible(true);
	window->requestFocus();
	screen->performLayout();
}

void AddCtrl::disposeWindow() {
	window->dispose();
	window = nullptr;
}

ftl::Configurable *AddCtrl::add(const std::string &uri) {
	try {
		if (io->feed()->sourceActive(uri)) {
			io->feed()->remove(uri);
		} else {
			io->feed()->add(uri);
		}
	} catch (const ftl::exception &e) {
		screen->showError("Exception", e.what());
	}
	return nullptr;
}

std::vector<std::string> AddCtrl::getHosts() {
	return std::move(io->feed()->knownHosts());
}

std::vector<std::string> AddCtrl::getGroups() {
	return std::move(io->feed()->availableGroups());
}

std::set<ftl::stream::SourceInfo> AddCtrl::getRecent() {
	return std::move(io->feed()->recentSources());
}

std::vector<std::string> AddCtrl::getNetSources() {
	return std::move(io->feed()->availableNetworkSources());
}

std::vector<std::string> AddCtrl::getFileSources() {
	return std::move(io->feed()->availableFileSources());
}

std::vector<std::string> AddCtrl::getDeviceSources() {
	return std::move(io->feed()->availableDeviceSources());
}

std::string AddCtrl::getSourceName(const std::string &uri) {
	return io->feed()->getName(uri);
}

bool AddCtrl::isSourceActive(const std::string &uri) {
	return io->feed()->sourceActive(uri);
}

AddCtrl::~AddCtrl() {
	// remove window?
}
