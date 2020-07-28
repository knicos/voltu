#include <sstream>

#include "visualization.hpp"
#include "widgets.hpp"
#include "intrinsicview.hpp"

#include "../../screen.hpp"
#include "../../widgets/window.hpp"

#include <opencv2/calib3d.hpp>

#include <nanogui/messagedialog.h>
#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <nanogui/button.h>
#include <nanogui/checkbox.h>
#include <nanogui/textbox.h>
#include <nanogui/label.h>

using ftl::codecs::Channel;

using ftl::gui2::Screen;
using ftl::gui2::View;
using ftl::gui2::FixedWindow;

using ftl::gui2::IntrinsicCalibrationStart;
using ftl::gui2::IntrinsicCalibration;
using ftl::gui2::IntrinsicCalibrationView;
using Mode = ftl::gui2::IntrinsicCalibrationView::Mode;

////////////////////////////////////////////////////////////////////////////////

template<typename T>
std::string to_string(T v, int precision = 2) {
	std::stringstream stream;
	stream << std::fixed << std::setprecision(precision) << v;
	return stream.str();
}

////////////////////////////////////////////////////////////////////////////////

class IntrinsicCalibrationView::CaptureWindow : public FixedWindow {
public:
	CaptureWindow(nanogui::Widget* parent, IntrinsicCalibrationView* view);
	virtual void draw(NVGcontext* ctx) override;

private:
	void update();
	IntrinsicCalibrationView* view_;
	IntrinsicCalibration* ctrl_;

	nanogui::Widget* channels_;

	int width_;
	int height_;
	double square_size_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class IntrinsicCalibrationView::ControlWindow : public FixedWindow {
public:
	ControlWindow(nanogui::Widget* parent, IntrinsicCalibrationView* view);
	virtual void draw(NVGcontext* ctx) override;

private:
	void updateCount();

	IntrinsicCalibrationView* view_;
	IntrinsicCalibration* ctrl_;

	nanogui::Label* txtnframes_;
	nanogui::Button* bcalibrate_;
	nanogui::Button* bsave_;
	nanogui::Button* bapply_;
	nanogui::Button* bresults_;
	nanogui::Button* bpause_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class IntrinsicCalibrationView::CalibrationWindow : public FixedWindow {
public:
	CalibrationWindow(nanogui::Widget* parent, IntrinsicCalibrationView* view);
	void update();
	virtual void draw(NVGcontext* ctx) override;

private:
	IntrinsicCalibrationView* view_;
	IntrinsicCalibration* ctrl_;

	nanogui::Label* status_;
	nanogui::Button* bcalibrate_;
	nanogui::FloatBox<double>* sensor_width_;
	nanogui::FloatBox<double>* sensor_height_;
	nanogui::FloatBox<double>* focal_length_;
	nanogui::CheckBox* reset_dist_;
	nanogui::CheckBox* reset_pp_;
	bool calibrating_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class IntrinsicCalibrationView::ResultWindow : public FixedWindow {
public:
	ResultWindow(nanogui::Widget* parent, IntrinsicCalibrationView* view);
	virtual void draw(NVGcontext* ctx) override;
	void update();

private:
	IntrinsicCalibrationView* view_;
	IntrinsicCalibration* ctrl_;

	nanogui::Button* bsave_;
	nanogui::Label* rms_;
	ftl::gui2::IntrinsicDetails* info_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


////////////////////////////////////////////////////////////////////////////////
//

IntrinsicCalibrationStart::IntrinsicCalibrationStart(ftl::gui2::Screen *parent, IntrinsicCalibration *ctrl) :
		ftl::gui2::View(parent), ctrl_(ctrl) {

	show_all_ = false;
	window_ = new FixedWindow(parent, std::string("Intrinsic Calibration"));
	window_->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Vertical,
									 nanogui::Alignment::Fill, 6, 12));

	auto* button_refresh = new nanogui::Button(window_->buttonPanel(), "", ENTYPO_ICON_CCW);
	button_refresh->setCallback([this](){ update(); });

	buttons_ = new nanogui::Widget(window_);
	buttons_->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Vertical,
									 nanogui::Alignment::Fill, 0, 8));

	auto bshow_all = new nanogui::CheckBox(window_, "Show all sources",
		[this](bool v){
			show_all_ = v;
			update();
	});
	bshow_all->setChecked(show_all_);

	window_->setFixedWidth(400);
	window_->setVisible(true);

	update();
}

IntrinsicCalibrationStart::~IntrinsicCalibrationStart() {
	window_->setVisible(false);
	if (parent()->getRefCount() > 0) {
		window_->dispose();
	}
}

void IntrinsicCalibrationStart::update() {
	while (buttons_->childCount() > 0) {
		buttons_->removeChild(buttons_->childCount() - 1);
	}

	for (const auto& [name, id] : ctrl_->listSources(show_all_)) {
		auto* button = new nanogui::Button(buttons_, name, ENTYPO_ICON_CAMERA);
		button->setCallback([ctrl = this->ctrl_, id](){
			ctrl->start(id);
		});
	}

	screen()->performLayout();
}

void IntrinsicCalibrationStart::draw(NVGcontext* ctx) {
	window_->center();
	View::draw(ctx);
}

////////////////////////////////////////////////////////////////////////////////
// Capture Window


void IntrinsicCalibrationView::CaptureWindow::update() {
	ctrl_->setChessboard({width_, height_}, square_size_);
}

IntrinsicCalibrationView::CaptureWindow::CaptureWindow(nanogui::Widget* parent, IntrinsicCalibrationView* view) :
	FixedWindow(parent, "Capture Options"), view_(view), ctrl_(view->ctrl_) {

	width_ = ctrl_->chessboardSize().width;
	height_ = ctrl_->chessboardSize().height;
	square_size_ = ctrl_->squareSize();

	setLayout(new nanogui::BoxLayout
		(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 6, 6));

	(new nanogui::Button(buttonPanel(), "", ENTYPO_ICON_CROSS))->setCallback(
		[view = view_]() {
		view->setMode(Mode::VIDEO);
	});

	// Capture parameters
	new nanogui::Label(this, "Select Camera");
	channels_ = new nanogui::Widget(this);
	channels_->setLayout(new nanogui::GridLayout
		(nanogui::Orientation::Horizontal, 2, nanogui::Alignment::Fill, 0, 0));
	auto* button_left = new nanogui::Button(channels_, "Left");
	button_left->setPushed(ctrl_->channel() == Channel::Left);
	button_left->setFlags(nanogui::Button::RadioButton);
	button_left->setCallback([ctrl = ctrl_, view=view_](){
		if (ctrl->channel() != Channel::Left) {
			ctrl->setChannel(Channel::Left);
			view->setUndistort(false);
		}
	});

	auto* button_right = new nanogui::Button(channels_, "Right");
	button_right->setFlags(nanogui::Button::RadioButton);
	button_right->setPushed(ctrl_->channel() == Channel::Right);
	button_right->setCallback([ctrl = ctrl_, view=view_](){
		if (ctrl->channel() != Channel::Right) {
			ctrl->setChannel(Channel::Right);
			view->setUndistort(false);
		}
	});
	button_right->setEnabled(ctrl_->hasChannel(Channel::Right));

	new nanogui::Label(this, "Capture interval");
	auto* interval = new nanogui::FloatBox<float>(this, ctrl_->frequency());
	interval->setEditable(true);
	interval->setFormat("[0-9]*\\.?[0-9]+");
	interval->setUnits("s");
	interval->setCallback([ctrl = this->ctrl_](float v){
		ctrl->setFrequency(v);
	});

	// Chessboard parameters
	auto* chessboard = new nanogui::Widget(this);
	chessboard->setLayout(new nanogui::BoxLayout
		(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 0, 4));

	// width
	new nanogui::Label(chessboard, "Chessboard width");
	auto* chessboard_size_x = new nanogui::IntBox<int>(chessboard, width_);
	chessboard_size_x->setEditable(true);
	chessboard_size_x->setFormat("[1-9][0-9]*");
	chessboard_size_x->setCallback([this](int v){
		width_ = max(0, v);
	});

	// height
	new nanogui::Label(chessboard, "Chessboard height");
	auto* chessboard_size_y = new nanogui::IntBox<int>(chessboard, height_);
	chessboard_size_y->setEditable(true);
	chessboard_size_y->setFormat("[1-9][0-9]*");
	chessboard_size_y->setCallback([this](int v){
		height_ = max(0, v);
	});

	// square size
	new nanogui::Label(chessboard, "Chessboard square size");
	auto* square_size = new nanogui::FloatBox<float>(chessboard, square_size_*1000.0);

	square_size->setEditable(true);
	square_size->setFormat("[0-9]*\\.?[0-9]+");
	square_size->setUnits("mm");
	square_size->setCallback([this](float v){
		square_size_ = v/1000.0;
	});

	auto* button_start = new nanogui::Button(this, "Start");
	button_start->setCallback([this]() {
		update();
		view_->setMode(Mode::CAPTURE_IMAGES);
	});
}

void IntrinsicCalibrationView::CaptureWindow::draw(NVGcontext* ctx) {
	channels_->childAt(1)->setEnabled(ctrl_->hasChannel(Channel::Right));
	FixedWindow::draw(ctx);
}

////////////////////////////////////////////////////////////////////////////////
// Control Window

IntrinsicCalibrationView::ControlWindow::ControlWindow(nanogui::Widget* parent, IntrinsicCalibrationView* view) :
	FixedWindow(parent, ""), view_(view), ctrl_(view->ctrl_) {

	setLayout(new nanogui::BoxLayout
		(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 6, 6));

	txtnframes_ = new nanogui::Label(this, "");
	updateCount();

	auto* buttons = new nanogui::Widget(this);
	buttons->setLayout(new nanogui::BoxLayout
		(nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 0, 0));

	auto* bback_ = new nanogui::Button(buttons, "", ENTYPO_ICON_ARROW_LEFT);
	bback_->setFixedWidth(40);
	bback_->setTooltip("Back to capture options");
	bback_->setCallback([this, button = bback_](){
		view_->setMode(Mode::CAPTURE_INIT);
	});

	bsave_ = new nanogui::Button(buttons, "", ENTYPO_ICON_SAVE);
	bsave_->setFixedWidth(40);
	bsave_->setTooltip("Save calibration");
	bsave_->setEnabled(ctrl_->calibrated());
	bsave_->setCallback([ctrl = ctrl_, view = view_](){
		ctrl->save();
		new nanogui::MessageDialog
			(view->screen(), nanogui::MessageDialog::Type::Information, "Calibration", "Calibration sent");
	});

	bapply_ = new nanogui::Button(buttons, "");
	bapply_->setFixedWidth(40);
	bapply_->setTooltip("Apply distortion correction");
	bapply_->setEnabled(ctrl_->calibrated());
	bapply_->setFlags(nanogui::Button::Flags::ToggleButton);
	bapply_->setPushed(view_->undistort());
	bapply_->setChangeCallback([button = bapply_, view = view_](bool v){
		view->setUndistort(v);
	});

	bresults_ = new nanogui::Button(buttons, "Details");
	bresults_->setFixedWidth(120);

	bresults_->setEnabled(ctrl_->calibrated());
	bresults_->setCallback([view = view_, button = bresults_]{
		view->setMode(Mode::RESULTS);
	});

	bpause_ = new nanogui::Button(buttons, "");
	bpause_->setFixedWidth(120);
	bpause_->setCallback([&ctrl = ctrl_](){
		// TODO: add buttons to browse captured images and allow deleting
		//		 images
		ctrl->setCapture(!ctrl->capturing());
	});

	bcalibrate_ = new nanogui::Button(buttons, "Calibrate");
	bcalibrate_->setFixedWidth(120);
	bcalibrate_->setCallback([view = view_, button = bcalibrate_](){
		view->setMode(Mode::CALIBRATION);
	});
}

void IntrinsicCalibrationView::ControlWindow::draw(NVGcontext* ctx) {
	if (ctrl_->capturing())	{ bpause_->setCaption("Pause"); }
	else 					{ bpause_->setCaption("Continue"); }
	//bcalibrate_->setEnabled(ctrl_->count() > 0);
	bresults_->setEnabled(ctrl_->calibrated());
	bsave_->setEnabled(ctrl_->calibrated());
	bapply_->setEnabled(ctrl_->calibrated());
	bapply_->setIcon(view_->undistort() ? ENTYPO_ICON_EYE : ENTYPO_ICON_EYE_WITH_LINE);
	bapply_->setPushed(view_->undistort());
	updateCount();
	FixedWindow::draw(ctx);
}

void IntrinsicCalibrationView::ControlWindow::updateCount() {
	txtnframes_->setCaption("Detected patterns: " +
							std::to_string(ctrl_->count()));
}
////////////////////////////////////////////////////////////////////////////////
// Calibration Window

IntrinsicCalibrationView::CalibrationWindow::CalibrationWindow(nanogui::Widget* parent, IntrinsicCalibrationView* view) :
		FixedWindow(parent, "Calibration"), view_(view), ctrl_(view->ctrl_) {

	calibrating_ = false;

	setLayout(new nanogui::BoxLayout
		(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 6, 6));

	(new nanogui::Button(buttonPanel(), "", ENTYPO_ICON_CROSS))->setCallback(
		[view = view_]() {
		view->setMode(Mode::VIDEO);
	});

	// sensor size
	new nanogui::Label(this, "Initial values");

	nanogui::GridLayout *grid_layout = new nanogui::GridLayout
		(nanogui::Orientation::Horizontal, 2, nanogui::Alignment::Fill, 0, 5);
	grid_layout->setColAlignment
		({nanogui::Alignment::Maximum, nanogui::Alignment::Fill});

	grid_layout->setSpacing(0, 10);
	auto* initial_values = new nanogui::Widget(this);
	initial_values->setLayout(grid_layout);

	new nanogui::Label(initial_values, "Sensor width");
	sensor_width_ = new nanogui::FloatBox<double>(initial_values, ctrl_->sensorSize().width);
	sensor_width_->setEditable(true);
	sensor_width_->setFormat("[0-9]*\\.?[0-9]+");
	sensor_width_->setUnits("mm");

	new nanogui::Label(initial_values, "Sensor height");
	sensor_height_ = new nanogui::FloatBox<double>(initial_values, ctrl_->sensorSize().height);
	sensor_height_->setEditable(true);
	sensor_height_->setFormat("[0-9]*\\.?[0-9]+");
	sensor_height_->setUnits("mm");

	new nanogui::Label(initial_values, "Focal length");
	focal_length_ = new nanogui::FloatBox<double>(initial_values, ctrl_->focalLength());
	focal_length_->setEditable(true);
	focal_length_->setFormat("[0-9]*\\.?[0-9]+");
	focal_length_->setUnits("mm");

	new nanogui::Label(initial_values, "Reset principal point");
	reset_pp_ = new nanogui::CheckBox(initial_values, "");
	reset_pp_->setChecked(false);

	new nanogui::Label(initial_values, "Reset distortion coefficients");
	reset_dist_ = new nanogui::CheckBox(initial_values, "");
	reset_dist_->setChecked(false);

	// flags
	new nanogui::Label(this, "Flags");
	new ftl::gui2::OpenCVFlagWidget(this, &(ctrl_->flags()), ctrl_->defaultFlags());
	status_ = new nanogui::Label(this, " ");

	bcalibrate_ = new nanogui::Button(this, "Run");
	bcalibrate_->setEnabled(false);
	bcalibrate_->setCallback([this](){
		if (!ctrl_->isBusy()) {
			ctrl_->setSensorSize({sensor_width_->value(), sensor_height_->value()});
			ctrl_->setFocalLength(focal_length_->value(), ctrl_->sensorSize());
			if (reset_pp_->checked()) { ctrl_->resetPrincipalPoint(); }
			if (reset_dist_->checked()) { ctrl_->resetDistortion(); }
			ctrl_->run();
			calibrating_ = true;
		}
	});
}

void IntrinsicCalibrationView::CalibrationWindow::update() {
	focal_length_->setValue(ctrl_->focalLength());
}

void IntrinsicCalibrationView::CalibrationWindow::draw(NVGcontext* ctx) {
	bool use_guess = ctrl_->flags().has(cv::CALIB_USE_INTRINSIC_GUESS);
	focal_length_->setEnabled(use_guess);
	reset_pp_->setEnabled(use_guess);
	reset_dist_->setEnabled(use_guess);

	if (ctrl_->isBusy()) {
		if (calibrating_) {
			auto dots = std::string(int(round(glfwGetTime())) % 4, '.');
			status_->setCaption("Calibrating " + dots);
		}
		else {
			status_->setCaption("Busy");
		}
	}
	else {
		status_->setCaption(" ");
	}
	bcalibrate_->setEnabled(!ctrl_->isBusy() && (ctrl_->count() > 0));
	if (calibrating_ && !ctrl_->isBusy()) {
		calibrating_ = false;
		view_->setUndistort(true);
		view_->setMode(Mode::RESULTS);
	}
	FixedWindow::draw(ctx);
}

////////////////////////////////////////////////////////////////////////////////
// Result window

IntrinsicCalibrationView::ResultWindow::ResultWindow(nanogui::Widget* parent, IntrinsicCalibrationView* view) :
	FixedWindow(parent, "Results"), view_(view), ctrl_(view->ctrl_) {

	setLayout(new nanogui::BoxLayout
		(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 8 , 8));

	(new nanogui::Button(buttonPanel(), "", ENTYPO_ICON_CROSS))->setCallback(
		[view = view_]() {
		view->setMode(Mode::VIDEO);
	});

	rms_ = new nanogui::Label(this, "");

	info_ = new ftl::gui2::IntrinsicDetails(this);

	bsave_ = new nanogui::Button(this, "Save");
	bsave_->setCallback([button = bsave_, ctrl = ctrl_](){
		ctrl->save();
		button->setCaption("Saved");
		button->setEnabled(false);
	});
}

void IntrinsicCalibrationView::ResultWindow::draw(NVGcontext* ctx) {
	nanogui::Window::draw(ctx);
}

void IntrinsicCalibrationView::ResultWindow::update() {
	if (!isnan(ctrl_->reprojectionError())) {
		rms_->setCaption("Reprojection error (RMS): " + to_string(ctrl_->reprojectionError()));
		rms_->setVisible(true);
	}
	else {
		rms_->setVisible(false);
	}
	info_->update(ctrl_->calibration());
	bsave_->setEnabled(true);
	bsave_->setCaption("Save");
}

////////////////////////////////////////////////////////////////////////////////

IntrinsicCalibrationView::IntrinsicCalibrationView(Screen* parent,
		IntrinsicCalibration* ctrl) : View(parent), ctrl_(ctrl) {

	undistort_ = false;

	imview_ = new ftl::gui2::FTLImageView(this);

	int w = 300;
	wcapture_ = new CaptureWindow(screen(), this);
	wcapture_->setFixedWidth(w);
	wcontrol_ = new ControlWindow(screen(), this);
	wcalibration_ = new CalibrationWindow(screen(), this);
	wcalibration_->setFixedWidth(w);
	wresults_ = new ResultWindow(screen(), this);
	wresults_->update();

	screen()->performLayout();
	setMode(Mode::CAPTURE_INIT);
}

IntrinsicCalibrationView::~IntrinsicCalibrationView() {
	wcapture_->setVisible(false);
	wcapture_->dispose();
	wcontrol_->setVisible(false);
	wcontrol_->dispose();
	wcalibration_->setVisible(false);
	wcalibration_->dispose();
	wresults_->setVisible(false);
	wresults_->dispose();
}

void IntrinsicCalibrationView::performLayout(NVGcontext *ctx) {
	auto sz = wcontrol_->size();
	wcontrol_->setPosition(
		nanogui::Vector2i(width() / 2 - sz[0]/2, height() - 30 - sz[1]));

	wcapture_->center();
	wcalibration_->center();
	wresults_->center();
	imview_->setSize(size());
	View::performLayout(ctx);
}

void IntrinsicCalibrationView::draw(NVGcontext *ctx) {
	if (ctrl_->hasFrame()) {
		bool was_valid = imview_->texture().isValid();
		if (undistort_) {
			auto frame = ctrl_->getFrameUndistort();
			imview_->copyFrom(frame);
		}
		else {
			auto frame = ctrl_->getFrame();
			imview_->copyFrom(frame);
		}
		if (!was_valid) {
			imview_->fit();
		}
	}
	View::draw(ctx);
	if (ctrl_->capturing()) {
		drawChessboardCorners(ctx, imview_, ctrl_->previousPoints());
	}
}

void IntrinsicCalibrationView::setMode(Mode m) {
	switch(m) {
		case Mode::CAPTURE_INIT:
			ctrl_->setCapture(false);
			wcapture_->setVisible(true);
			wcontrol_->setVisible(false);
			wcalibration_->setVisible(false);
			wresults_->setVisible(false);
			break;

		case Mode::CAPTURE_IMAGES:
			ctrl_->setCapture(true);
			wcapture_->setVisible(false);
			wcontrol_->setVisible(true);
			wcalibration_->setVisible(false);
			wresults_->setVisible(false);
			break;

		case Mode::VIDEO:
			ctrl_->setCapture(false);
			wcapture_->setVisible(false);
			wcontrol_->setVisible(true);
			wcalibration_->setVisible(false);
			wresults_->setVisible(false);
			break;

		case Mode::CALIBRATION:
			ctrl_->setCapture(false);
			wcapture_->setVisible(false);
			wcontrol_->setVisible(false);
			wcalibration_->update();
			wcalibration_->setVisible(true);
			wresults_->setVisible(false);
			break;

		case Mode::RESULTS:
			ctrl_->setCapture(false);
			wcapture_->setVisible(false);
			wcontrol_->setVisible(false);
			wcalibration_->setVisible(false);
			wresults_->setVisible(true);
			wresults_->update();
			break;
	}
	screen()->performLayout();
}
