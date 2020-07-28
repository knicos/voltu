#include <nanogui/screen.h>
#include <nanogui/layout.h>
#include <nanogui/button.h>
#include <nanogui/vscrollpanel.h>
#include <ftl/utility/string.hpp>

#include <ftl/codecs/touch.hpp>

#include "camera.hpp"

#include "../modules/camera.hpp"
#include "../modules/config.hpp"
#include "../modules/statistics.hpp"

#include "../widgets/popupbutton.hpp"

#include <loguru.hpp>

using ftl::gui2::Camera;
using ftl::gui2::FixedWindow;
using ftl::gui2::MediaPanel;
using ftl::gui2::CameraView;
using ftl::gui2::PopupButton;
using ftl::gui2::VolumeButton;

using ftl::codecs::Channel;

// ==== Record Options =========================================================

class RecordOptions : public nanogui::Window {
public:
	RecordOptions(nanogui::Widget *parent, Camera* ctrl);
	virtual ~RecordOptions();

	void show(const std::function<void(bool)> &cb);

private:
	Camera* ctrl_;
	std::list<std::tuple<nanogui::CheckBox*,ftl::codecs::Channel>> channels_;
	std::function<void(bool)> callback_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

RecordOptions::RecordOptions(nanogui::Widget *parent, Camera* ctrl)
 : nanogui::Window(parent, "Recording"), ctrl_(ctrl) {

	using namespace nanogui;

	//setFixedWidth(300);
	setLayout(new GroupLayout(15, 6, 14, 10));
	setPosition(Vector2i(parent->width()/2.0f - 100.0f, parent->height()/2.0f - 100.0f));
	setVisible(false);

	auto close = new nanogui::Button(buttonPanel(), "", ENTYPO_ICON_CROSS);
	close->setTheme(dynamic_cast<ftl::gui2::Screen*>(screen())->getTheme("window_dark"));
	close->setBackgroundColor(theme()->mWindowHeaderGradientBot);
	close->setCallback([this](){
		setVisible(false);
		if (callback_) callback_(false);
	});

	auto filename_box = new TextBox(this, "test.ftl");
	filename_box->setEditable(true);

	VScrollPanel *vscroll = new VScrollPanel(this);
	vscroll->setFixedHeight(150);
	Widget *scroll = new Widget(vscroll);
	scroll->setLayout(new GridLayout(Orientation::Horizontal, 3));
	//auto *label = new Label(vscroll, "Select Channels:", "sans-bold");

	// Add all available channels as checkboxes
	// TODO: Refresh this list on show
	auto channels = ctrl_->allAvailableChannels();
	for (auto c : channels) {
		// Skip channels that can't be encoded
		if (int(c) < 32) {
			switch (c) {
			case Channel::Colour	:
			case Channel::Colour2	:
			case Channel::Depth		:
			case Channel::Depth2	:
			case Channel::ColourHighRes :
			case Channel::Colour2HighRes : break;
			default: continue;
			}
		}

		auto check = new CheckBox(scroll, ftl::codecs::name(c));
		switch (c) {
		case Channel::Colour		:
		case Channel::Pose			:
		case Channel::Capabilities	:
		case Channel::Calibration	:
		case Channel::MetaData		: check->setChecked(true); break;
		default: break;
		}

		if (c == Channel::Calibration) {
			check->setEnabled(false);
		}

		channels_.emplace_back(check, c);
	}

	auto *button_panel = new Widget(this);
	button_panel->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 6));

	auto start = new Button(button_panel, "Record");
	start->setCallback([this, filename_box]() {
		std::unordered_set<ftl::codecs::Channel> selection;
		for (auto &s : channels_) {
			if (std::get<0>(s)->checked()) {
				selection.emplace(std::get<1>(s));
			}
		}

		if (selection.size() > 0) {
			ctrl_->startRecording(filename_box->value(), selection);
			setVisible(false);
		}

		if (callback_) callback_(true);
	});

	auto stream = new Button(button_panel, "Stream");
	stream->setCallback([this]() {
		std::unordered_set<ftl::codecs::Channel> selection;
		for (auto &s : channels_) {
			if (std::get<0>(s)->checked()) {
				selection.emplace(std::get<1>(s));
			}
		}

		if (selection.size() > 0) {
			ctrl_->startStreaming(selection);
			setVisible(false);
		}

		if (callback_) callback_(true);
	});

	auto closebut = new Button(button_panel, "Cancel");
	closebut->setCallback([this]() {
		setVisible(false);
		if (callback_) callback_(false);
	});

	auto advanced = new Button(button_panel, "Advanced");
	advanced->setEnabled(false);

	screen()->performLayout();
}

RecordOptions::~RecordOptions() {

}

void RecordOptions::show(const std::function<void(bool)> &cb) {
	setVisible(true);
	callback_ = cb;
}

// === MediaPanel ==============================================================

class MediaPanel : public FixedWindow {
public:
	MediaPanel(nanogui::Widget *parent, Camera* ctrl);
	virtual ~MediaPanel();

	void setAvailableChannels(const std::unordered_set<ftl::codecs::Channel> &channels);
	void setActiveChannel(ftl::codecs::Channel c);

	void draw(NVGcontext *ctx) override;

	/** add button to position. */
	nanogui::Button* addButton(int position = -1);

	PopupButton* button_channels;
	VolumeButton* button_volume;

private:
	std::vector<nanogui::Widget*> buttons(); // channel buttons
	Camera* ctrl_;
	RecordOptions *record_opts_=nullptr;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

MediaPanel::MediaPanel(nanogui::Widget *parent, ftl::gui2::Camera* ctrl) :
	ftl::gui2::FixedWindow(parent, ""), ctrl_(ctrl) {

	LOG(INFO) << __func__ << " (" << this << ")";
	using namespace nanogui;

	record_opts_ = new RecordOptions(screen(), ctrl);

	setLayout(new BoxLayout(Orientation::Horizontal,
									Alignment::Middle, 5, 10));

	auto theme = dynamic_cast<ftl::gui2::Screen*>(screen())->getTheme("media");
	this->setTheme(theme);

	// Volume control
	button_volume = new ftl::gui2::VolumeButton(this);
	button_volume->setValue(ctrl_->volume());
	button_volume->setCallback([ctrl = ctrl_](float v){ ctrl->setVolume(v); });

	// Pause/Unpause
	auto button_pause = new Button(this, "", ENTYPO_ICON_CONTROLLER_PAUS);
	if (ctrl->isPaused()) {
		button_pause->setIcon(ENTYPO_ICON_CONTROLLER_PLAY);
	}

	button_pause->setCallback([ctrl = ctrl_ ,button_pause]() {
		ctrl->setPaused(!ctrl->isPaused());

		if (ctrl->isPaused()) {
			button_pause->setIcon(ENTYPO_ICON_CONTROLLER_PLAY);
		} else {
			button_pause->setIcon(ENTYPO_ICON_CONTROLLER_PAUS);
		}
	});

	// Record
	/*auto button_record = new ftl::gui2::PopupButton(this, "", ENTYPO_ICON_CONTROLLER_RECORD);
	button_record->setSide(Popup::Side::Right);
	button_record->setChevronIcon(0);

	auto rec_popup = button_record->popup();
	rec_popup->setLayout(new GroupLayout());

	{
		auto button = new Button(rec_popup, "Record to File");
		//button->setFlags(Button::RadioButton);
		//button->setVisible(true);
		button->setCallback([this, button_record]() {
			if (!ctrl_->isRecording()) {
				button_record->setTextColor(nanogui::Color(1.0f,0.1f,0.1f,1.0f));
				button_record->setPushed(false);
				ctrl_->startRecording("test.ftl");
			}
		});
	}

	button_record->setCallback([this, button_record]() {
		if (ctrl_->isRecording()) {
			button_record->setTextColor(nanogui::Color(1.0f,1.0f,1.0f,1.0f));
			button_record->setPushed(false);
			ctrl_->stopRecording();
		}
	});*/

	// Record
	auto button_record = new Button(this, "", ENTYPO_ICON_CONTROLLER_RECORD);
	button_record->setCallback([this, button_record]() {
		if (record_opts_->visible()) return;

		if (ctrl_->isRecording()) {
			ctrl_->stopRecording();
			button_record->setTextColor(nanogui::Color(1.0f,1.0f,1.0f,1.0f));
		} else {
			record_opts_->show([button_record](bool rec) {
				if (rec) button_record->setTextColor(nanogui::Color(1.0f,0.1f,0.1f,1.0f));
			});
		}
	});

	// Channel select. Creates buttons for 32 channels and sets available ones
	// visible (a bit of a hack, only used here and setAvailableChannels())

	button_channels = new ftl::gui2::PopupButton(this, "", ENTYPO_ICON_LAYERS);
	button_channels->setSide(Popup::Side::Right);
	button_channels->setChevronIcon(0);

	auto popup = button_channels->popup();
	popup->setLayout(new GroupLayout());

	for (int i=0; i < 32; ++i) {
		ftl::codecs::Channel c = static_cast<ftl::codecs::Channel>(i);
		auto button = new Button(popup, ftl::codecs::name(c));
		button->setFlags(Button::RadioButton);
		button->setVisible(false);
		button->setCallback([this,c]() {
			ctrl_->setChannel(c);
			setActiveChannel(c);
		});
	}

	setAvailableChannels(ctrl_->availableChannels());

	// Settings
	auto button_config = new Button(this, "", ENTYPO_ICON_COG);

	button_config->setCallback([ctrl = ctrl_]() {
		auto uri = ctrl->getActiveSourceURI();
		if (uri.size() > 0) ctrl->screen->getModule<ftl::gui2::ConfigCtrl>()->show(uri);
		else ctrl->screen->showError("Error", "This source does not have any settings");
	});
}

MediaPanel::~MediaPanel() {
	if (parent()->getRefCount() > 0) record_opts_->dispose();
}

void MediaPanel::draw(NVGcontext *ctx) {
	auto size = this->size();
	setPosition(
		nanogui::Vector2i(	screen()->width() / 2 - size[0]/2,
							screen()->height() - 30 - size[1]));

	FixedWindow::draw(ctx);
}

std::vector<nanogui::Widget*> MediaPanel::buttons() {

	auto popup = button_channels->popup();

	if (popup->childCount() != 32) {
		LOG(ERROR) << "Wrong number of buttons!";
	}
	return popup->children();
}

void MediaPanel::setAvailableChannels(const std::unordered_set<Channel> &channels) {

	const auto &button = buttons();
	bool update = false;

	for (int i = 0; i < 32; ++i) {
		ftl::codecs::Channel c = static_cast<ftl::codecs::Channel>(i);
		bool visible = channels.count(c) > 0;
		update |= (visible != button[i]->visible());
		button[i]->setVisible(visible);
	}

	if (update) {
		auto popup = button_channels->popup();
		screen()->performLayout();
		popup->setAnchorHeight(popup->height() - 20);
	}
}

void MediaPanel::setActiveChannel(Channel c) {
	auto button = dynamic_cast<nanogui::Button*>
		(buttons()[static_cast<size_t>(c)]);

	button->setVisible(true);
	button->setPushed(true);
}

nanogui::Button* MediaPanel::addButton(int pos) {
	auto* button = new nanogui::Button(this, "", 0);
	if (pos >= 0) {
		mChildren.pop_back();
		mChildren.insert(mChildren.begin() + pos, button);
	}
	performLayout(screen()->nvgContext());
	return button;
}

// ==== CameraView =============================================================

CameraView::CameraView(ftl::gui2::Screen* parent, ftl::gui2::Camera* ctrl) :
		View(parent), enable_zoom_(false), enable_pan_(false), ctrl_(ctrl) {

	imview_ = new ftl::gui2::FTLImageView(this);
	panel_ = new ftl::gui2::MediaPanel(screen(), ctrl);

	auto *mod = ctrl_->screen->getModule<ftl::gui2::Statistics>();
	if (ctrl_->isMovable()) {
		imview_->setCursor(nanogui::Cursor::Hand);
		mod->setCursor(nanogui::Cursor::Hand);
	} else {
		imview_->setCursor(nanogui::Cursor::Crosshair);
		mod->setCursor(nanogui::Cursor::Crosshair);
	}

	auto theme = dynamic_cast<ftl::gui2::Screen*>(screen())->getTheme("toolbutton");
	//this->setTheme(theme);

	context_menu_ = new nanogui::Window(parent, "");
	context_menu_->setVisible(false);
	context_menu_->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Vertical));
	context_menu_->setTheme(theme);

	auto *button = new nanogui::Button(context_menu_, "Capture Image");
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
	});
}

CameraView::~CameraView() {
	if (parent()->getRefCount() > 0) {
		// segfault without this check; nanogui already deleted windows?
		// should be fixed in nanogui
		panel_->dispose();
	}

	if (context_menu_->parent()->getRefCount() > 0) {
		context_menu_->setVisible(false);
		context_menu_->dispose();
	}
}

void CameraView::refresh() {
	bool was_valid = imview_->texture().isValid();

	if (ctrl_->hasFrame()) {
		imview_->copyFrom(ctrl_->getFrame());
	}
	if (!was_valid && imview_->texture().isValid()) {
		screen()->performLayout();
	}
}

void CameraView::setZoom(bool v) {
	enable_zoom_ = v;
	imview_->setFixedScale(!v);
	if (!v) {
		imview_->setScale(1.0f);
	}
}

void CameraView::setPan(bool v) {
	enable_pan_ = v;
	imview_->setFixedOffset(!v);
	if (!v) {
		imview_->fit();
	}
}

bool CameraView::mouseMotionEvent(const Eigen::Vector2i &p, const Eigen::Vector2i &rel, int button, int modifiers) {
	//if (button == 1) {
		auto pos = imview_->imageCoordinateAt((p - mPos + rel).cast<float>());
		if (pos.x() >= 0.0f && pos.y() >= 0.0f) {
			ctrl_->touch(0, ftl::codecs::TouchType::MOUSE_LEFT, pos.x(), pos.y(), 0.0f, (button > 0) ? 255 : 0);

			//LOG(INFO) << "Depth at " << pos.x() << "," << pos.y() << " = " << ctrl_->depthAt(pos.x(), pos.y());
		}
		return true;
	//}
	return false;
}

bool CameraView::mouseButtonEvent(const Eigen::Vector2i &p, int button, bool down, int modifiers) {
	//LOG(INFO) << "mouseButtonEvent: " << p << " - " << button;
	if (button == 0) {
		auto pos = imview_->imageCoordinateAt((p - mPos).cast<float>());
		if (pos.x() >= 0.0f && pos.y() >= 0.0f) {
			ctrl_->touch(0, ftl::codecs::TouchType::MOUSE_LEFT, pos.x(), pos.y(), 0.0f, (down) ? 255 : 0);
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

void CameraView::draw(NVGcontext*ctx) {
	if (ctrl_->hasFrame()) {
		try {
			// TODO: Select shader to flip if VR capability found...
			imview_->copyFrom(ctrl_->getFrame());
		}
		catch (std::exception& e) {
			gui()->showError("Exception", e.what());
		}
	}
	View::draw(ctx);

	ctrl_->drawOverlay(ctx);

	auto mouse = screen()->mousePos();
	auto pos = imview_->imageCoordinateAt((mouse - mPos).cast<float>());
	float d = ctrl_->depthAt(pos.x(), pos.y());

	if (d > 0.0f) {
		nvgText(ctx, mouse.x()+25.0f, mouse.y()+20.0f, (to_string_with_precision(d,2) + std::string("m")).c_str(), nullptr);
	}
}

void CameraView::performLayout(NVGcontext* ctx) {
	imview_->setSize(size());
	if (!(enable_zoom_ && enable_pan_)) {
		imview_->fit();
	}
	View::performLayout(ctx);
}