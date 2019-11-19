#include "media_panel.hpp"
#include "screen.hpp"
#include "camera.hpp"

#include <nanogui/layout.h>
#include <nanogui/button.h>
#include <nanogui/popupbutton.h>
#include <nanogui/entypo.h>

#ifdef HAVE_LIBARCHIVE
#include "ftl/rgbd/snapshot.hpp"
#endif

using ftl::gui::MediaPanel;
using ftl::codecs::Channel;

MediaPanel::MediaPanel(ftl::gui::Screen *screen) : nanogui::Window(screen, ""), screen_(screen) {
	using namespace nanogui;

	paused_ = false;
	disable_switch_channels_ = false;

	setLayout(new BoxLayout(Orientation::Horizontal,
									Alignment::Middle, 5, 10));

	auto size = Vector2i(400, 60);
	//setFixedSize(size);
	setPosition(Vector2i(screen->width() / 2 - size[0]/2, screen->height() - 30 - size[1]));

	auto button = new Button(this, "", ENTYPO_ICON_EDIT);
	button->setTooltip("Edit camera properties");
	button->setCallback([this]() {
		auto *cam = screen_->activeCamera();
		if (cam) cam->showPoseWindow();
	});

	auto recordbutton = new PopupButton(this, "", ENTYPO_ICON_CONTROLLER_RECORD);
	recordbutton->setTooltip("Record");
	recordbutton->setSide(Popup::Side::Right);
	recordbutton->setChevronIcon(0);
	auto recordpopup = recordbutton->popup();
	recordpopup->setLayout(new GroupLayout());
	recordpopup->setTheme(screen->toolbuttheme);
	recordpopup->setAnchorHeight(150);
	auto itembutton = new Button(recordpopup, "2D snapshot (.png)");
	itembutton->setCallback([this,recordbutton]() {
		screen_->activeCamera()->snapshot();
		recordbutton->setPushed(false);
	});
	itembutton = new Button(recordpopup, "Virtual camera recording (.ftl)");
	itembutton->setCallback([this,recordbutton]() {
		std::cout << "Toggling video recording in itembutton callback." << '\n';
		screen_->activeCamera()->toggleVideoRecording();
		recordbutton->setCallback([this,recordbutton]() {
			std::cout << "Toggling video recording in recordbutton callback." << '\n';
			screen_->activeCamera()->toggleVideoRecording();
			recordbutton->setCallback([]() {});
			recordbutton->setTextColor(nanogui::Color(1.0f,1.0f,1.0f,1.0f));

			// Prevents the popup from being opened, though it is shown while the button
			// is being pressed.
			recordbutton->setPushed(false);
		});
		recordbutton->setTextColor(nanogui::Color(1.0f,0.1f,0.1f,1.0f));
		recordbutton->setPushed(false);
	});
	itembutton = new Button(recordpopup, "3D scene recording (.ftl)");
	itembutton->setCallback([this,recordbutton]() {
		auto tag = screen_->activeCamera()->source()->get<std::string>("uri");
		if (tag) {
			auto tagvalue = tag.value();
			auto configurables = ftl::config::findByTag(tagvalue);
			if (configurables.size() > 0) {
				ftl::Configurable *configurable = configurables[0];
				configurable->set("record", true);
				recordbutton->setTextColor(nanogui::Color(1.0f,0.1f,0.1f,1.0f));
				recordbutton->setCallback([this,recordbutton,configurable]() {
					configurable->set("record", false);
					recordbutton->setCallback([]() {});
					recordbutton->setTextColor(nanogui::Color(1.0f,1.0f,1.0f,1.0f));
					recordbutton->setPushed(false);
				});
			}
		}
		recordbutton->setPushed(false);
	});
	itembutton = new Button(recordpopup, "Detailed recording options");

	button = new Button(this, "", ENTYPO_ICON_CONTROLLER_STOP);
	button->setCallback([this]() {
		screen_->setActiveCamera(nullptr);
	});

	button = new Button(this, "", ENTYPO_ICON_CONTROLLER_PAUS);
	button->setCallback([this,button]() {
		paused_ = !paused_;
		screen_->control()->pause();
		if (paused_) {
			button->setIcon(ENTYPO_ICON_CONTROLLER_PLAY);
		} else {
			button->setIcon(ENTYPO_ICON_CONTROLLER_PAUS);
		}
	});

	//button = new Button(this, "", ENTYPO_ICON_CONTROLLER_RECORD);

	/* Doesn't work at the moment
 #ifdef HAVE_LIBARCHIVE
	auto button_snapshot = new Button(this, "", ENTYPO_ICON_IMAGES);
	button_snapshot->setTooltip("Screen capture");
	button_snapshot->setCallback([this] {
	ftl::gui::Camera *cam = screen_->activeCamera();
	if (!cam) return;

	try {
		char timestamp[18];
			std::time_t t=std::time(NULL);
			std::strftime(timestamp, sizeof(timestamp), "%F-%H%M%S", std::localtime(&t));
			auto writer = ftl::rgbd::SnapshotWriter(std::string(timestamp) + ".tar.gz");
			cv::Mat rgb, depth;
			cam->source()->getFrames(rgb, depth);
			writer.addSource(	cam->source()->getURI(),
								cam->source()->parameters(),
								cam->source()->getPose());
			writer.addRGBD(0, rgb, depth);
		}
		catch(std::runtime_error) {
			LOG(ERROR) << "Snapshot failed (file error)";
		}
	});
#endif

	
	// not very useful (l/r)

	auto button_dual = new Button(this, "", ENTYPO_ICON_MAP);
	button_dual->setCallback([this]() {
		screen_->setDualView(!screen_->getDualView());
	});
	*/

#ifdef HAVE_OPENVR
	if (this->screen_->hasVR()) {
		auto button_vr = new Button(this, "VR");
		button_vr->setFlags(Button::ToggleButton);
		button_vr->setChangeCallback([this, button_vr](bool state) {
			if (!screen_->useVR()) {
				if (screen_->switchVR(true) == true) {
					button_vr->setTextColor(nanogui::Color(0.5f,0.5f,1.0f,1.0f));
					this->button_channels_->setEnabled(false);
				}
			}
			else {
				if (screen_->switchVR(false) == false) {
					button_vr->setTextColor(nanogui::Color(1.0f,1.0f,1.0f,1.0f));
					this->button_channels_->setEnabled(true);
				}
			}
		});
	}
#endif

	button_channels_ = new PopupButton(this, "", ENTYPO_ICON_LAYERS);
	button_channels_->setSide(Popup::Side::Right);
	button_channels_->setChevronIcon(ENTYPO_ICON_CHEVRON_SMALL_RIGHT);
	Popup *popup = button_channels_->popup();
	popup->setLayout(new GroupLayout());
	popup->setTheme(screen->toolbuttheme);
	popup->setAnchorHeight(150);

	button = new Button(popup, "Left");
	button->setFlags(Button::RadioButton);
	button->setPushed(true);
	button->setCallback([this]() {
		ftl::gui::Camera *cam = screen_->activeCamera();
		if (cam) {
			cam->setChannel(Channel::Left);
		}
	});

	right_button_ = new Button(popup, "Right");
	right_button_->setFlags(Button::RadioButton);
	right_button_->setCallback([this]() {
		ftl::gui::Camera *cam = screen_->activeCamera();
		if (cam) {
			cam->setChannel(Channel::Right);
		}
	});

	depth_button_ = new Button(popup, "Depth");
	depth_button_->setFlags(Button::RadioButton);
	depth_button_->setCallback([this]() {
		ftl::gui::Camera *cam = screen_->activeCamera();
		if (cam) {
			cam->setChannel(Channel::Depth);
		}
	});

	auto *popbutton = new PopupButton(popup, "More");
	popbutton->setSide(Popup::Side::Right);
	popbutton->setChevronIcon(ENTYPO_ICON_CHEVRON_SMALL_RIGHT);
	popup = popbutton->popup();
	popup->setLayout(new GroupLayout());
	popup->setTheme(screen->toolbuttheme);
	popup->setAnchorHeight(150);

	button = new Button(popup, "Deviation");
	button->setFlags(Button::RadioButton);
	button->setCallback([this]() {
		ftl::gui::Camera *cam = screen_->activeCamera();
		if (cam) {
			cam->setChannel(Channel::Deviation);
		}
	});

	button = new Button(popup, "Normals");
	button->setFlags(Button::RadioButton);
	button->setCallback([this]() {
		ftl::gui::Camera *cam = screen_->activeCamera();
		if (cam) {
			cam->setChannel(Channel::ColourNormals);
		}
	});

	button = new Button(popup, "Flow");
	button->setFlags(Button::RadioButton);
	button->setCallback([this]() {
		ftl::gui::Camera *cam = screen_->activeCamera();
		if (cam) {
			cam->setChannel(Channel::Flow);
		}
	});

	button = new Button(popup, "Confidence");
	button->setFlags(Button::RadioButton);
	button->setCallback([this]() {
		ftl::gui::Camera *cam = screen_->activeCamera();
		if (cam) {
			cam->setChannel(Channel::Confidence);
		}
	});

	button = new Button(popup, "Energy");
	button->setFlags(Button::RadioButton);
	button->setCallback([this]() {
		ftl::gui::Camera *cam = screen_->activeCamera();
		if (cam) {
			cam->setChannel(Channel::Energy);
		}
	});

	button = new Button(popup, "Density");
	button->setFlags(Button::RadioButton);
	button->setCallback([this]() {
		ftl::gui::Camera *cam = screen_->activeCamera();
		if (cam) {
			cam->setChannel(Channel::Density);
		}
	});

}

MediaPanel::~MediaPanel() {

}

// Update button enabled status
void MediaPanel::cameraChanged() {
	ftl::gui::Camera *cam = screen_->activeCamera();
	if (cam) {
		if (cam->source()->hasCapabilities(ftl::rgbd::kCapStereo)) {
			right_button_->setEnabled(true);
		} else {
			right_button_->setEnabled(false);
		}
	}
}
