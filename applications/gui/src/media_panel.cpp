#include "media_panel.hpp"
#include "screen.hpp"
#include "record_window.hpp"

#include <nanogui/layout.h>
#include <nanogui/button.h>
#include <nanogui/popupbutton.h>
#include <nanogui/entypo.h>

#ifdef HAVE_LIBARCHIVE
#include "ftl/rgbd/snapshot.hpp"
#endif

using ftl::gui::MediaPanel;
using ftl::codecs::Channel;

MediaPanel::MediaPanel(ftl::gui::Screen *screen, ftl::gui::SourceWindow *sourceWindow) : nanogui::Window(screen, ""), screen_(screen), sourceWindow_(sourceWindow) {
	using namespace nanogui;

	paused_ = false;
	disable_switch_channels_ = false;
	record_mode_ = RecordMode::None;

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

	recordbutton_ = new PopupButton(this, "", ENTYPO_ICON_CONTROLLER_RECORD);
	recordbutton_->setTooltip("Record");
	recordbutton_->setSide(Popup::Side::Right);
	recordbutton_->setChevronIcon(0);
	auto recordpopup = recordbutton_->popup();
	recordpopup->setLayout(new GroupLayout());
	recordpopup->setTheme(screen->toolbuttheme);
	recordpopup->setAnchorHeight(150);
	auto itembutton = new Button(recordpopup, "2D snapshot (.png)");
	itembutton->setCallback([this]() {
		_startRecording(RecordMode::Snapshot2D);
		recordbutton_->setPushed(false);
	});
	itembutton = new Button(recordpopup, "Virtual camera recording (.ftl)");
	itembutton->setCallback([this]() {
		_startRecording(RecordMode::Video2D);
		recordbutton_->setTextColor(nanogui::Color(1.0f,0.1f,0.1f,1.0f));
		recordbutton_->setPushed(false);
	});
	itembutton = new Button(recordpopup, "3D scene snapshot (.ftl)");
	itembutton->setCallback([this]() {
		_startRecording(RecordMode::Snapshot3D);
		recordbutton_->setPushed(false);
	});
	itembutton = new Button(recordpopup, "3D scene recording (.ftl)");
	itembutton->setCallback([this]() {
		_startRecording(RecordMode::Video3D);
		recordbutton_->setTextColor(nanogui::Color(1.0f,0.1f,0.1f,1.0f));
		recordbutton_->setPushed(false);
	});
	itembutton = new Button(recordpopup, "Detailed recording options");
	itembutton->setCallback([this,sourceWindow] {
		auto record_window = new RecordWindow(screen_, screen_, sourceWindow->getCameras(), this);
		record_window->setTheme(screen_->windowtheme);
		recordbutton_->setPushed(false);
		recordbutton_->setEnabled(false);
	});

	recordbutton_->setCallback([this](){
		if (record_mode_ != RecordMode::None) {
			_stopRecording();
			recordbutton_->setTextColor(nanogui::Color(1.0f,1.0f,1.0f,1.0f));
			recordbutton_->setPushed(false);
		}
	});

	button = new Button(this, "", ENTYPO_ICON_CONTROLLER_STOP);
	button->setCallback([this]() {
		screen_->setActiveCamera(nullptr);
	});

	button = new Button(this, "", ENTYPO_ICON_CONTROLLER_PAUS);
	button->setCallback([this,button]() {
		//paused_ = !paused_;
		paused_ = !(bool)ftl::config::get("[reconstruction]/controls/paused");
		ftl::config::update("[reconstruction]/controls/paused", paused_);
		if (paused_) {
			button->setIcon(ENTYPO_ICON_CONTROLLER_PLAY);
		} else {
			button->setIcon(ENTYPO_ICON_CONTROLLER_PAUS);
		}
	});
	
	// not very useful (l/r)

	/*auto button_dual = new Button(this, "", ENTYPO_ICON_MAP);
	button_dual->setCallback([this]() {
		screen_->setDualView(!screen_->getDualView());
	});
	*/

#ifdef HAVE_OPENVR
	if (this->screen_->isHmdPresent()) {
		auto button_vr = new Button(this, "VR");
		button_vr->setFlags(Button::ToggleButton);
		button_vr->setChangeCallback([this, button_vr](bool state) {
			if (!screen_->isVR()) {
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

	for (int i=0; i<=2; ++i) {
		ftl::codecs::Channel c = static_cast<ftl::codecs::Channel>(i);
		button = new Button(popup, ftl::codecs::name(c));
		button->setFlags(Button::RadioButton);
		//button->setPushed(true);
		button->setVisible(false);
		button->setCallback([this,c]() {
			ftl::gui::Camera *cam = screen_->activeCamera();
			if (cam) {
				cam->setChannel(c);
			}
		});
		channel_buttons_[i] = button;
	}

	auto *popbutton = new PopupButton(popup, "More");
	popbutton->setSide(Popup::Side::Right);
	popbutton->setChevronIcon(ENTYPO_ICON_CHEVRON_SMALL_RIGHT);
	popup = popbutton->popup();
	popup->setLayout(new GroupLayout());
	popup->setTheme(screen->toolbuttheme);
	popup->setAnchorHeight(150);

	for (int i=3; i<32; ++i) {
		ftl::codecs::Channel c = static_cast<ftl::codecs::Channel>(i);
		button = new Button(popup, ftl::codecs::name(c));
		button->setFlags(Button::RadioButton);
		//button->setPushed(true);
		button->setVisible(false);
		button->setCallback([this,c]() {
			ftl::gui::Camera *cam = screen_->activeCamera();
			if (cam) {
				cam->setChannel(c);
			}
		});
		channel_buttons_[i] = button;
	}
}

MediaPanel::~MediaPanel() {

}

void MediaPanel::_startRecording(MediaPanel::RecordMode mode) {
	char timestamp[18];
	std::time_t t=std::time(NULL);
	std::strftime(timestamp, sizeof(timestamp), "%F-%H%M%S", std::localtime(&t));

	std::string filename(timestamp);
	switch(mode) {
	case RecordMode::Snapshot2D		: filename += ".png"; break;
	case RecordMode::Snapshot3D		:
	case RecordMode::Video3D		: filename += ".ftl"; break;
	case RecordMode::Video2D		: filename += ".ftl"; break;
	}

	if (mode == RecordMode::Video3D) {
		record_mode_ = mode;
		sourceWindow_->recordVideo(filename);
	} else if (mode == RecordMode::Snapshot2D) {
		screen_->activeCamera()->snapshot(filename);
	} else if (mode == RecordMode::Video2D) {
		record_mode_ = mode;
		screen_->activeCamera()->startVideoRecording(filename);
	}
}

void MediaPanel::_stopRecording() {
	if (record_mode_ == RecordMode::Video3D) {
		sourceWindow_->stopRecordingVideo();
	} else if (record_mode_ == RecordMode::Video2D) {
		screen_->activeCamera()->stopVideoRecording();
	}
	record_mode_ = RecordMode::None;
}

// Update button enabled status
void MediaPanel::cameraChanged() {
	ftl::gui::Camera *cam = screen_->activeCamera();
	if (cam) {
		auto channels = cam->availableChannels();
		for (int i=0; i<32; ++i) {
			if (channels.has(static_cast<ftl::codecs::Channel>(i))) {
				channel_buttons_[i]->setVisible(true);
			} else {
				channel_buttons_[i]->setVisible(false);
			}

			if (cam->getChannel() == static_cast<ftl::codecs::Channel>(i)) {
				channel_buttons_[i]->setPushed(true);
			} else {
				channel_buttons_[i]->setPushed(false);
			}
		}
	}
}

void MediaPanel::recordWindowClosed() {
	recordbutton_->setEnabled(true);
}