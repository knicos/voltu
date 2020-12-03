/**
 * @file thumbnails.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Sebastian Hahta
 */

#pragma once
#include "../view.hpp"

#include "../widgets/imageview.hpp"

#include <nanogui/glcanvas.h>
#include <nanogui/glutil.h>
#include <nanogui/imageview.h>

namespace ftl {
namespace gui2 {

class ThumbnailsController;
class ThumbView;

class Thumbnails : public View {
public:
	Thumbnails(Screen *parent, ThumbnailsController *controller);
	virtual ~Thumbnails();

	virtual void draw(NVGcontext *ctx) override;

	bool mouseButtonEvent(const nanogui::Vector2i &p, int button, bool down, int modifiers) override;

private:
	void updateThumbnails();
	void addTab(unsigned int fsid);

	struct FSThumbnails {
		int64_t timestamp;
		nanogui::Widget* panel;
		std::vector<ThumbView*> thumbnails;
	};

	std::mutex mtx_;
	ftl::gui2::ThumbnailsController *ctrl_;
	nanogui::TabWidget* tabwidget_;

	std::map<unsigned int, FSThumbnails> thumbnails_;

	nanogui::Vector2i thumbsize_ = nanogui::Vector2i(320,180);

	nanogui::Window *context_menu_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
}
