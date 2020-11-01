#include <nanogui/screen.h>
#include <nanogui/layout.h>
#include <nanogui/button.h>
#include <nanogui/vscrollpanel.h>
#include <ftl/utility/string.hpp>

#include "disparityview.hpp"

#include "../../modules/dev/developer.hpp"
#include "../../modules/config.hpp"

#include "../../widgets/popupbutton.hpp"

#include <loguru.hpp>

using ftl::gui2::DisparityDev;
using ftl::gui2::FixedWindow;
using ftl::gui2::DisparityView;
using ftl::gui2::PopupButton;
using ftl::gui2::Tools;
using ftl::gui2::ToolGroup;

using ftl::codecs::Channel;

// ==== CameraView =============================================================

DisparityView::DisparityView(ftl::gui2::Screen* parent, ftl::gui2::DisparityDev* ctrl) :
		View(parent), ctrl_(ctrl),
		stereoim_(nullptr) {

	//imview_ = new ftl::gui2::FTLImageView(this);
	//panel_ = new ftl::gui2::MediaPanel(screen(), ctrl, this);
	//tools_ = new ftl::gui2::ToolPanel(screen(), ctrl, this);

	stereoim_ = new StereoImageView(this);
	imview_ = stereoim_->right();

	//imview_->setFlipped(ctrl->isVR());

	/*auto *mod = ctrl_->screen->getModule<ftl::gui2::Statistics>();
	if (ctrl_->isMovable()) {
		imview_->setCursor(nanogui::Cursor::Hand);
		mod->setCursor(nanogui::Cursor::Hand);
	} else {
		imview_->setCursor(nanogui::Cursor::Crosshair);
		mod->setCursor(nanogui::Cursor::Crosshair);
	}*/

	auto theme = dynamic_cast<ftl::gui2::Screen*>(screen())->getTheme("toolbutton");
	//this->setTheme(theme);

	context_menu_ = new nanogui::Window(parent, "");
	context_menu_->setVisible(false);
	context_menu_->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Vertical));
	context_menu_->setTheme(theme);

	screen()->performLayout();

	/*auto *button = new nanogui::Button(context_menu_, "Capture Image");
	button->setCallback([this]() {
		char timestamp[18];
		std::time_t t=std::time(NULL);
		std::strftime(timestamp, sizeof(timestamp), "%F-%H%M%S", std::localtime(&t));
		context_menu_->setVisible(false);
		ctrl_->snapshot(std::string(timestamp)+std::string(".png"));
	});

	button = new nanogui::Button(context_menu_, "Settings");
	button->setCallback([this, button]() {
		context_menu_->setVisible(false);
		ctrl_->screen->getModule<ftl::gui2::ConfigCtrl>()->show(ctrl_->getID());
	});*/

	/*tools_->setAvailable({
		Tools::SELECT_POINT,
		Tools::OVERLAY,
		Tools::PAN,
		Tools::ZOOM_FIT,
		Tools::ZOOM_IN,
		Tools::ZOOM_OUT,
		Tools::CENTRE_VIEW,
		Tools::INSPECT_POINT
	});

	tools_->addCallback([this](ftl::gui2::Tools tool) {
		switch (tool) {
		case Tools::OVERLAY		: ctrl_->toggleOverlay(); return true;
		case Tools::ZOOM_FIT		: imview_->fit(); return true;
		case Tools::CENTRE_VIEW	: imview_->center(); return true;
		//case CameraTools::ZOOM_OUT		: imview_->zoom(-1, imview_->sizeF() / 2); return true;
		//case CameraTools::ZOOM_IN		: imview_->zoom(1, imview_->sizeF() / 2); return true;
		default: return false;
		}
	});*/
}

DisparityView::~DisparityView() {
	if (parent()->getRefCount() > 0) {
		// segfault without this check; nanogui already deleted windows?
		// should be fixed in nanogui
		//panel_->dispose();
		//tools_->dispose();
	}

	if (context_menu_->parent()->getRefCount() > 0) {
		context_menu_->setVisible(false);
		context_menu_->dispose();
	}
}

void DisparityView::refresh() {
	bool was_valid = imview_->texture().isValid();

	if (ctrl_->hasFrame()) {
		ctrl_->generate();
		//imview_->copyFrom(ctrl_->getFrame());
		//stereoim_->left()->copyFrom(ctrl_->getFrame(Channel::Left));
		imview_->copyFrom(ctrl_->getFeatureImageRight(ftl::disparity::ColourFeatures::Feature::ALL));
		stereoim_->left()->copyFrom(ctrl_->getFeatureImageLeft(ftl::disparity::ColourFeatures::Feature::ALL));
	}
	if (!was_valid && imview_->texture().isValid()) {
		screen()->performLayout();
	}
}

bool DisparityView::mouseMotionEvent(const Eigen::Vector2i &p, const Eigen::Vector2i &rel, int button, int modifiers) {
	//if (button == 1) {

		/*if (tools_->isActive(Tools::SELECT_POINT)) {
			auto pos = imview_->imageCoordinateAt((p - mPos + rel).cast<float>());
			if (pos.x() >= 0.0f && pos.y() >= 0.0f) {
				ctrl_->touch(0, ftl::codecs::TouchType::MOUSE_LEFT, pos.x(), pos.y(), 0.0f, (button > 0) ? 255 : 0);

				//LOG(INFO) << "Depth at " << pos.x() << "," << pos.y() << " = " << ctrl_->depthAt(pos.x(), pos.y());
			}
		}*/
		return true;
	//}
	return false;
}

bool DisparityView::mouseButtonEvent(const Eigen::Vector2i &p, int button, bool down, int modifiers) {
	//LOG(INFO) << "mouseButtonEvent: " << p << " - " << button;
	if (button == 0) {
		if (down) {
			auto pos = imview_->imageCoordinateAt((p - mPos).cast<float>());
			LOG(INFO) << "Use focal point at " << pos.x() << "," << pos.y();

		}

		context_menu_->setVisible(false);
		return true;
	} else if (button == 1) {
		if (!down) {
			context_menu_->setPosition(p - mPos);
			context_menu_->setVisible(true);
			return true;
		}
	} else {
		context_menu_->setVisible(false);
	}
	return false;
}

void DisparityView::draw(NVGcontext*ctx) {
	using namespace nanogui;

	if (ctrl_->hasFrame()) {
		ctrl_->generate();

		try {
			// TODO: Select shader to flip if VR capability found...
			imview_->copyFrom(ctrl_->getFeatureImageRight(ftl::disparity::ColourFeatures::Feature::ALL));
			if (stereoim_) {
				stereoim_->left()->copyFrom(ctrl_->getFeatureImageLeft(ftl::disparity::ColourFeatures::Feature::ALL));
			}
		}
		catch (std::exception& e) {
			gui()->showError("Exception", e.what());
		}


		/*try {
			// TODO: Select shader to flip if VR capability found...
			imview_->copyFrom(ctrl_->getFrame());
			if (stereoim_) {
				stereoim_->left()->copyFrom(ctrl_->getFrame(Channel::Left));
			}
		}
		catch (std::exception& e) {
			gui()->showError("Exception", e.what());
		}*/
	}
	View::draw(ctx);

	auto osize = imview_->scaledImageSizeF();
	//ctrl_->drawOverlay(ctx, screen()->size().cast<float>(), osize, imview_->offset());

	/*if (tools_->isActive(Tools::INSPECT_POINT)) {
		auto mouse = screen()->mousePos();
		auto pos = imview_->imageCoordinateAt((mouse - mPos).cast<float>());
		float d = ctrl_->depthAt(pos.x(), pos.y());

		if (d > 0.0f) {
			nvgText(ctx, mouse.x()+25.0f, mouse.y()+20.0f, (to_string_with_precision(d,2) + std::string("m")).c_str(), nullptr);
		}
	}*/
}

void DisparityView::performLayout(NVGcontext* ctx) {
	if (stereoim_) {
		stereoim_->setFixedSize(size());
		//if (!(enable_zoom_ && enable_pan_)) {
			stereoim_->fit();
		//}
	}
	else {
		imview_->setSize(size());
		//if (!(enable_zoom_ && enable_pan_)) {
			imview_->fit();
		//}
	}
	View::performLayout(ctx);
}
