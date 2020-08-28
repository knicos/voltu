#include "thumbnails.hpp"
#include "../modules/thumbnails.hpp"
#include <ftl/utility/gltexture.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/cudaarithm.hpp>

#include <ftl/operators/antialiasing.hpp>
#include <ftl/cuda/normals.hpp>
#include <ftl/render/colouriser.hpp>
#include <ftl/cuda/transform.hpp>
#include <ftl/operators/gt_analysis.hpp>
#include <ftl/operators/poser.hpp>
#include <ftl/cuda/colour_cuda.hpp>
#include <ftl/streams/parsers.hpp>
#include <ftl/rgbd/frame.hpp>

#include <nanogui/label.h>
#include <nanogui/tabwidget.h>
#include <nanogui/vscrollpanel.h>
#include <nanogui/layout.h>
#include <nanogui/popup.h>

#include <loguru.hpp>

using ftl::gui2::ThumbView;
using ftl::gui2::Thumbnails;
using ftl::utility::GLTexture;
using ftl::gui2::ThumbnailsController;

using ftl::codecs::Channel;
using ftl::data::FrameID;

class ThumbView : public ftl::gui2::ImageView {
public:
	ThumbView(nanogui::Widget *parent, ThumbnailsController *control, ftl::data::FrameID id, const std::string &name);
	virtual ~ThumbView() {}

	virtual bool mouseButtonEvent(const nanogui::Vector2i &p, int button, bool down, int modifiers) override;
	virtual void draw(NVGcontext *ctx) override;

	void setName(const std::string &str) { name_ = str; }
	void update(ftl::rgbd::Frame& frame, Channel c);

private:
	ThumbnailsController *ctrl_;
	GLTexture texture_;
	const ftl::data::FrameID id_;
	std::string name_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

ThumbView::ThumbView(nanogui::Widget *parent, ThumbnailsController *control, ftl::data::FrameID id, const std::string &name) :
		ftl::gui2::ImageView(parent), ctrl_(control), id_(id), name_(name) {
	setCursor(nanogui::Cursor::Hand);
	setFixedOffset(true);
	setFixedScale(true);
}

bool ThumbView::mouseButtonEvent(const nanogui::Vector2i &p, int button, bool down, int modifiers) {
	if (button == 0) {
		if (!down) {
			ctrl_->show_camera(id_);
		}
	}
	return true;
}

void ThumbView::update(ftl::rgbd::Frame &frame, Channel c) {
	if (!frame.hasChannel(c)) {
		return;
	}

	const auto &vf = frame.get<ftl::rgbd::VideoFrame>(c);

	if (vf.isGPU()) {
		texture_.copyFrom(vf.getGPU());
	} else {
		texture_.copyFrom(vf.getCPU());
	}
	if (texture_.isValid()) {
		bindImage(texture_.texture());
	}
}

void ThumbView::draw(NVGcontext *ctx) {
	fit();
	// Image
	ftl::gui2::ImageView::draw(ctx);
	// Label
	nvgScissor(ctx, mPos.x(), mPos.y(), mSize.x(), mSize.y());
	nvgFontSize(ctx, 14);
	nvgFontFace(ctx, "sans-bold");
	nvgTextAlign(ctx, NVG_ALIGN_CENTER);
	nvgText(ctx, mPos.x() + mSize.x()/2.0f, mPos.y()+mSize.y() - 18, name_.c_str(), NULL);
	nvgResetScissor(ctx);
}

////////////////////////////////////////////////////////////////////////////////

Thumbnails::Thumbnails(ftl::gui2::Screen *parent, ftl::gui2::ThumbnailsController *control) :
		View(parent), ctrl_(control), tabwidget_(nullptr) {

	tabwidget_ = new nanogui::TabWidget(this);
	tabwidget_->setFixedSize(size());

	context_menu_ = new nanogui::Window(parent, "");
	context_menu_->setVisible(false);
	context_menu_->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Vertical));

	auto *button = new nanogui::Button(context_menu_, "Remove");
	button->setCallback([this]() {
		int ix = tabwidget_->activeTab();
		LOG(INFO) << "REMOVE FSID " << ix;

		tabwidget_->removeTab(ix);
		thumbnails_.erase(ix);
		context_menu_->setVisible(false);
		ctrl_->removeFrameset(ix);
		//screen()->performLayout();
	});
}


Thumbnails::~Thumbnails() {
	if (context_menu_->parent()->getRefCount() > 0) {
		context_menu_->setVisible(false);
		context_menu_->dispose();
	}
}

bool Thumbnails::mouseButtonEvent(const nanogui::Vector2i &p, int button, bool down, int modifiers) {
	bool r = View::mouseButtonEvent(p, button, down, modifiers);

	if (!r) {
		if (button == 1) {
			if (!down) {
				context_menu_->setPosition(p - mPos);
				context_menu_->setVisible(true);
				return true;
			}
		} else {
			context_menu_->setVisible(false);
		}
	}

	return true;
}

void Thumbnails::updateThumbnails() {
	const Channel channel = Channel::Colour;
	bool perform_layout = false;
	auto framesets = ctrl_->getFrameSets();
	for (auto& fs : framesets) {
		unsigned int fsid = fs->frameset();

		// create new tab if necessary
		if (thumbnails_.count(fsid) == 0) {
			if (fs->frames.size() == 0) {
				// setting layout to widget without any children will crash
				// nanogui, skip
				continue;
			}

			auto* tab = tabwidget_->createTab(fs->name());
			tab->setLayout(new nanogui::BoxLayout
				(nanogui::Orientation::Vertical, nanogui::Alignment::Middle, 40));
			auto* panel = new nanogui::Widget(tab);
			panel->setLayout(
				new nanogui::GridLayout(nanogui::Orientation::Horizontal, 3,
										nanogui::Alignment::Middle, 0, 10));

			thumbnails_[fsid] = {0, panel, {}};
			perform_layout = true;
		}

		auto& thumbs = thumbnails_[fsid];
		while (thumbs.thumbnails.size() < fs->frames.size()) {
			int source = thumbs.thumbnails.size();
			auto &frame = fs->frames[source];

			perform_layout = true;

			std::string name = frame.name();

			auto* thumbnail = new ThumbView(thumbs.panel, ctrl_, FrameID(fsid, source), name);
			thumbnail->setFixedSize(thumbsize_);
			thumbs.thumbnails.push_back(thumbnail);
		}

		if (fs->timestamp() > thumbs.timestamp) {
			for(size_t i = 0; i < fs->frames.size(); i++) {
				thumbs.thumbnails[i]->update((*fs)[i].cast<ftl::rgbd::Frame>(), channel);
			}
			thumbs.timestamp = fs->timestamp();
		}
	}
	if (perform_layout) {
		screen()->performLayout();
	}
}

void Thumbnails::draw(NVGcontext *ctx) {
	tabwidget_->setFixedSize(size());
	updateThumbnails();
	View::draw(ctx);
}

