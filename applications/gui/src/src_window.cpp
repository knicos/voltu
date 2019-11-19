#include "src_window.hpp"

#include "screen.hpp"
#include "camera.hpp"

#include <nanogui/imageview.h>
#include <nanogui/textbox.h>
#include <nanogui/slider.h>
#include <nanogui/combobox.h>
#include <nanogui/label.h>
#include <nanogui/opengl.h>
#include <nanogui/glutil.h>
#include <nanogui/screen.h>
#include <nanogui/layout.h>
#include <nanogui/vscrollpanel.h>

#ifdef HAVE_LIBARCHIVE
#include "ftl/rgbd/snapshot.hpp"
#endif

#include "thumbview.hpp"

using ftl::gui::SourceWindow;
using ftl::gui::Screen;
using ftl::rgbd::Source;
using std::string;
using std::vector;
using ftl::config::json_t;

SourceWindow::SourceWindow(ftl::gui::Screen *screen)
		: nanogui::Window(screen, ""), screen_(screen) {
	setLayout(new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 20, 5));

	using namespace nanogui;
	
	//if (!screen->root()->get<json_t>("sources")) {
	//	screen->root()->getConfig()["sources"] = json_t::array();
	//}

	//src_ = ftl::create<Source>(ctrl->getRoot(), "source", ctrl->getNet());

	//Widget *tools = new Widget(this);
	//    tools->setLayout(new BoxLayout(Orientation::Horizontal,
	//                                   Alignment::Middle, 0, 6));

	new Label(this, "Select Camera","sans-bold",20);

	//auto select = new ComboBox(this, available_);
	//select->setCallback([this,select](int ix) {
		//src_->set("uri", available_[ix]);
		// TODO(Nick) Check camera exists first
	//	screen_->setActiveCamera(cameras_[available_[ix]]);
	//	LOG(INFO) << "Change source: " << ix;
	//});

	auto vscroll = new VScrollPanel(this);
	ipanel_ = new Widget(vscroll);
	ipanel_->setLayout(new GridLayout(nanogui::Orientation::Horizontal, 2,
		nanogui::Alignment::Middle, 0, 5));
	//ipanel_ = new ImageView(vscroll, 0);

	screen->net()->onConnect([this](ftl::net::Peer *p) {
		UNIQUE_LOCK(mutex_, lk);
		_updateCameras(screen_->net()->findAll<string>("list_streams"));
	});

	UNIQUE_LOCK(mutex_, lk);

	std::vector<ftl::rgbd::Source*> srcs = ftl::createArray<ftl::rgbd::Source>(screen_->root(), "sources", screen_->net());
	for (auto *src : srcs) {
		available_.push_back(src->getURI());
	}

	_updateCameras(screen_->control()->getNet()->findAll<string>("list_streams"));
}

std::vector<ftl::gui::Camera*> SourceWindow::getCameras() {
	auto cameras = std::vector<ftl::gui::Camera*>(cameras_.size());
	for (const auto &kv : cameras_) {
		cameras.push_back(kv.second);
	}
	return cameras;
}

void SourceWindow::_updateCameras(const vector<string> &netcams) {
	for (auto s : netcams) {
		if (cameras_.find(s) == cameras_.end()) {
			available_.push_back(s);
			json_t srcjson;
			srcjson["uri"] = s;
			screen_->root()->getConfig()["sources"].push_back(srcjson);
		}
	}

	std::vector<ftl::rgbd::Source*> srcs = ftl::createArray<ftl::rgbd::Source>(screen_->root(), "sources", screen_->net());
	for (auto *src : srcs) {
		if (cameras_.find(src->getURI()) == cameras_.end()) {
			LOG(INFO) << "Making camera: " << src->getURI();

			auto *cam = new ftl::gui::Camera(screen_, src);
			cameras_[src->getURI()] = cam;
		} else {
			//LOG(INFO) << "Camera already exists: " << s;
		}
	}

	refresh_thumbs_ = true;
	if (thumbs_.size() != available_.size()) {
		thumbs_.resize(available_.size());
	}
}

SourceWindow::~SourceWindow() {

}

void SourceWindow::draw(NVGcontext *ctx) {
	if (refresh_thumbs_) {
		UNIQUE_LOCK(mutex_, lk);
		//refresh_thumbs_ = false;

		for (size_t i=0; i<thumbs_.size(); ++i) {
			cv::Mat t;
			auto *cam = cameras_[available_[i]];
			if (cam) {
				if (cam->thumbnail(t)) {
					thumbs_[i].update(t);
				} else {
					refresh_thumbs_ = true;
				}
			}

			if ((size_t)ipanel_->childCount() < i+1) {
				new ftl::gui::ThumbView(ipanel_, screen_, cam);
			}
			if (thumbs_[i].isValid()) dynamic_cast<nanogui::ImageView*>(ipanel_->childAt(i))->bindImage(thumbs_[i].texture());
		}

		// TODO(Nick) remove excess image views

		center();
	}

	nanogui::Window::draw(ctx);
}
