/**
 * @file thumbnails.cpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Sebastian Hahta
 */

#include "thumbnails.hpp"
#include "dev/developer.hpp"
#include "../views/thumbnails.hpp"

#include "camera.hpp"

#include <ftl/codecs/channels.hpp>

#include <nanogui/entypo.h>

using ftl::codecs::Channel;
using ftl::gui2::ThumbnailsController;

void ThumbnailsController::init() {
	auto button = screen->addButton(ENTYPO_ICON_HOME);
	button->setTooltip("Home");
	button->setCallback([this, button](){
		button->setPushed(false);
		activate();
	});
	button->setVisible(true);
}

void ThumbnailsController::activate() {
	show_thumbnails();
}

ThumbnailsController::~ThumbnailsController() {

}

void ThumbnailsController::removeFrameset(uint32_t id) {
	{
		std::unique_lock<std::mutex> lk(mtx_);
		framesets_.erase(id);
	}
	io->feed()->remove(id);
}

void ThumbnailsController::show_thumbnails() {
	auto thumb_view = new ftl::gui2::Thumbnails(screen, this);

	auto* filter = io->feed()->filter({Channel::Colour});
	filter->on(
		[this, thumb_view](const ftl::data::FrameSetPtr& fs){
			{
				std::unique_lock<std::mutex> lk(mtx_);
				framesets_[fs->frameset()] = fs;
			}
			screen->redraw();
			return true;
	});

	thumb_view->onClose([filter](){
		filter->remove();
	});

	screen->setView(thumb_view);
}

std::vector<ftl::data::FrameSetPtr> ThumbnailsController::getFrameSets() {
	std::unique_lock<std::mutex> lk(mtx_);
	std::vector<ftl::data::FrameSetPtr> framesets;
	framesets.reserve(framesets_.size());

	for (auto& [k, v] : framesets_) {
		std::ignore = k;
		framesets.push_back(v);
	}

	return framesets;
}

void ThumbnailsController::show_camera(ftl::data::FrameID id) {
	auto *dispdev = screen->getModuleNoExcept<ftl::gui2::DisparityDev>();
	if (dispdev) {
		dispdev->activate(id);
	} else {
		auto* camera = screen->getModule<ftl::gui2::Camera>();
		camera->activate(id);
	}
}
