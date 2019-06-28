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

MediaPanel::MediaPanel(ftl::gui::Screen *screen) : nanogui::Window(screen, ""), screen_(screen) {
    using namespace nanogui;

    paused_ = false;
    writer_ = nullptr;

    setLayout(new BoxLayout(Orientation::Horizontal,
									Alignment::Middle, 5, 10));

    auto size = Vector2i(400, 60);
    //setFixedSize(size);
    setPosition(Vector2i(screen->width() / 2 - size[0]/2, screen->height() - 30 - size[1]));

	Theme *mediatheme = new Theme(*theme());
	mediatheme->mIconScale = 1.2f;
	mediatheme->mWindowDropShadowSize = 0;
	mediatheme->mWindowFillFocused = nanogui::Color(45, 150);
	mediatheme->mWindowFillUnfocused = nanogui::Color(45, 80);
	mediatheme->mButtonGradientTopUnfocused = nanogui::Color(0,0);
	mediatheme->mButtonGradientBotUnfocused = nanogui::Color(0,0);
	mediatheme->mButtonGradientTopFocused = nanogui::Color(80,230);
	mediatheme->mButtonGradientBotFocused = nanogui::Color(80,230);
	mediatheme->mIconColor = nanogui::Color(255,255);
    mediatheme->mTextColor = nanogui::Color(1.0f,1.0f,1.0f,1.0f);
	mediatheme->mBorderDark = nanogui::Color(0,0);
	mediatheme->mBorderMedium = nanogui::Color(0,0);
	mediatheme->mBorderLight = nanogui::Color(0,0);
	mediatheme->mDropShadow = nanogui::Color(0,0);
	mediatheme->mButtonFontSize = 30;

	setTheme(mediatheme);

    auto button = new Button(this, "", ENTYPO_ICON_EDIT);
	button->setTooltip("Edit camera properties");
    button->setCallback([this]() {
        auto *cam = screen_->activeCamera();
        if (cam) cam->showPoseWindow();
    });

    button = new Button(this, "", ENTYPO_ICON_CONTROLLER_RECORD);
    button->setFlags(Button::ToggleButton);
    button->setChangeCallback([this,button](bool state) {
        if (state){
            auto *cam = screen_->activeCamera();

            button->setTextColor(nanogui::Color(1.0f,0.1f,0.1f,1.0f));
            char timestamp[18];
			std::time_t t=std::time(NULL);
			std::strftime(timestamp, sizeof(timestamp), "%F-%H%M%S", std::localtime(&t));
			writer_ = new ftl::rgbd::SnapshotStreamWriter(std::string(timestamp) + ".tar.gz", 1000 / 25);
            writer_->addSource(cam->source());
            writer_->start();
        } else {
            button->setTextColor(nanogui::Color(1.0f,1.0f,1.0f,1.0f));
            if (writer_) {
                writer_->stop();
                delete writer_;
                writer_ = nullptr;
            }
        }
        //if (state) ... start
        //else ... stop
    });

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
			if (!writer.addCameraParams("0", cam->source()->getPose(), cam->source()->parameters()) || !writer.addCameraRGBD(
					"0", // TODO
					rgb,
					depth
				)) {
				LOG(ERROR) << "Snapshot failed";
			}
		}
		catch(std::runtime_error) {
			LOG(ERROR) << "Snapshot failed (file error)";
		}
	});
#endif

    auto popbutton = new PopupButton(this, "", ENTYPO_ICON_LAYERS);
    popbutton->setSide(Popup::Side::Right);
	popbutton->setChevronIcon(ENTYPO_ICON_CHEVRON_SMALL_RIGHT);
    Popup *popup = popbutton->popup();
    popup->setLayout(new GroupLayout());
    popup->setAnchorHeight(100);

    button = new Button(popup, "Left");
    button->setFlags(Button::RadioButton);
    button->setPushed(true);
    button->setCallback([this]() {
        ftl::gui::Camera *cam = screen_->activeCamera();
        if (cam) {
            cam->setChannel(ftl::rgbd::kChanLeft);
        }
    });

    button = new Button(popup, "Depth");
    button->setFlags(Button::RadioButton);
    button->setCallback([this]() {
        ftl::gui::Camera *cam = screen_->activeCamera();
        if (cam) {
            cam->setChannel(ftl::rgbd::kChanDepth);
        }
    });

    button = new Button(popup, "Deviation");
    button->setFlags(Button::RadioButton);
    button->setCallback([this]() {
        ftl::gui::Camera *cam = screen_->activeCamera();
        if (cam) {
            cam->setChannel(ftl::rgbd::kChanDeviation);
        }
    });
}

MediaPanel::~MediaPanel() {

}
