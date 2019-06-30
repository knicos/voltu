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

	auto label = new Label(this, "Select Camera","sans-bold",20);

	available_ = screen_->control()->getNet()->findAll<string>("list_streams");
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
		available_ = screen_->net()->findAll<string>("list_streams");
		//select->setItems(available_);
		_updateCameras();
	});

	UNIQUE_LOCK(mutex_, lk);
	_updateCameras();

	/*new Label(this, "Source Options","sans-bold");

	auto tools = new Widget(this);
    tools->setLayout(new BoxLayout(Orientation::Horizontal,
                                       Alignment::Middle, 0, 6));

	auto button_rgb = new Button(tools, "RGB");
	button_rgb->setTooltip("RGB left image");
	button_rgb->setFlags(Button::RadioButton);
	button_rgb->setPushed(true);
	button_rgb->setChangeCallback([this](bool state) { mode_ = Mode::rgb; });

	auto button_depth = new Button(tools, "Depth");
	button_depth->setFlags(Button::RadioButton);
	button_depth->setChangeCallback([this](bool state) { mode_ = Mode::depth; });

	auto button_stddev = new Button(tools, "SD. 25");
	button_stddev->setTooltip("Standard Deviation over 25 frames");
	button_stddev->setFlags(Button::RadioButton);
	button_stddev->setChangeCallback([this](bool state) { mode_ = Mode::stddev; });

	//auto button_pose = new Button(this, "Adjust Pose", ENTYPO_ICON_COMPASS);
	//button_pose->setCallback([this]() {
	//	auto posewin = new PoseWindow(screen_, screen_->control(), src_->getURI());
	//	posewin->setTheme(theme());
	//});

#ifdef HAVE_LIBARCHIVE
	auto button_snapshot = new Button(this, "Snapshot", ENTYPO_ICON_IMAGES);
	button_snapshot->setCallback([this] {
		try {
			/*char timestamp[18];
			std::time_t t=std::time(NULL);
			std::strftime(timestamp, sizeof(timestamp), "%F-%H%M%S", std::localtime(&t));
			auto writer = ftl::rgbd::SnapshotWriter(std::string(timestamp) + ".tar.gz");
			cv::Mat rgb, depth;
			this->src_->getFrames(rgb, depth);
			if (!writer.addCameraRGBD(
					"0", // TODO
					rgb,
					depth,
					this->src_->getPose(),
					this->src_->parameters()
				)) {
				LOG(ERROR) << "Snapshot failed";
			}
		}
		catch(std::runtime_error) {
			LOG(ERROR) << "Snapshot failed (file error)";
		}
	});
#endif

	//auto imageView = new VirtualCameraView(this);
	//cam.view = imageView;
	//imageView->setGridThreshold(20);
	//imageView->setSource(src_);
	//image_ = imageView;*/
}

void SourceWindow::_updateCameras() {
	refresh_thumbs_ = true;
	if (thumbs_.size() != available_.size()) {
		thumbs_.resize(available_.size());
	}

	for (auto s : available_) {
		if (cameras_.find(s) == cameras_.end()) {
			json_t srcjson;
			srcjson["uri"] = s;
			screen_->root()->getConfig()["sources"].push_back(srcjson);
			std::vector<ftl::rgbd::Source*> srcs = ftl::createArray<ftl::rgbd::Source>(screen_->root(), "sources", screen_->net());
			auto *src = srcs[srcs.size()-1];

			LOG(INFO) << "Making camera: " << src->getURI();

			auto *cam = new ftl::gui::Camera(screen_, src);
			cameras_[s] = cam;
		} else {
			LOG(INFO) << "Camera already exists: " << s;
		}
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
				if (cam->source()->thumbnail(t)) {
					thumbs_[i].update(t);
				} else {
					refresh_thumbs_ = true;
				}
			}

			if (ipanel_->childCount() < i+1) {
				new ftl::gui::ThumbView(ipanel_, screen_, cam);
			}
			if (thumbs_[i].isValid()) dynamic_cast<nanogui::ImageView*>(ipanel_->childAt(i))->bindImage(thumbs_[i].texture());
		}

		// TODO(Nick) remove excess image views

		center();
	}

	nanogui::Window::draw(ctx);
}
