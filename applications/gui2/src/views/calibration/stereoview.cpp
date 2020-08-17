#include <sstream>

#include "visualization.hpp"
#include "widgets.hpp"
#include "stereoview.hpp"


#include "../../screen.hpp"
#include "../../widgets/window.hpp"


#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <nanogui/button.h>
#include <nanogui/checkbox.h>
#include <nanogui/textbox.h>
#include <nanogui/label.h>
#include <nanogui/tabwidget.h>

using ftl::codecs::Channel;

using ftl::gui2::Screen;
using ftl::gui2::View;
using ftl::gui2::FixedWindow;

using ftl::gui2::StereoCalibrationStart;
using ftl::gui2::StereoCalibration;
using ftl::gui2::StereoCalibrationView;
using Mode = ftl::gui2::StereoCalibrationView::Mode;

////////////////////////////////////////////////////////////////////////////////

template<typename T>
std::string to_string(T v, int precision = 2) {
	std::stringstream stream;
	stream << std::fixed << std::setprecision(precision) << v;
	return stream.str();
}

////////////////////////////////////////////////////////////////////////////////

class StereoCalibrationView::CaptureWindow : public FixedWindow {
public:
	CaptureWindow(nanogui::Widget* parent, StereoCalibrationView* view);
	virtual void draw(NVGcontext* ctx) override;

private:
	void update();
	StereoCalibrationView* view_;
	StereoCalibration* ctrl_;
	int width_;
	int height_;
	double square_size_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class StereoCalibrationView::ControlWindow : public FixedWindow {
public:
	ControlWindow(nanogui::Widget* parent, StereoCalibrationView* view);
	virtual void draw(NVGcontext* ctx) override;

private:
	void updateCount();

	StereoCalibrationView* view_;
	StereoCalibration* ctrl_;

	nanogui::Label* txtnframes_;
	nanogui::Button* bcalibrate_;
	nanogui::Button* bresults_;
	nanogui::Button* bpause_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class StereoCalibrationView::CalibrationWindow : public FixedWindow {
public:
	CalibrationWindow(nanogui::Widget* parent, StereoCalibrationView* view);
	virtual void draw(NVGcontext* ctx) override;
	double sensorWidth() { return sensor_width_->value(); }
	double sensorHeight() { return sensor_width_->value(); }

private:
	StereoCalibrationView* view_;
	StereoCalibration* ctrl_;

	nanogui::Button* bcalibrate_;
	nanogui::FloatBox<double>* sensor_width_;
	nanogui::FloatBox<double>* sensor_height_;
	bool calibrating_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class StereoCalibrationView::ResultWindow : public FixedWindow {
public:
	ResultWindow(nanogui::Widget* parent, StereoCalibrationView* view);
	virtual void performLayout(NVGcontext* ctx) override;
	virtual void draw(NVGcontext* ctx) override;
	void update();

private:
	StereoCalibrationView* view_;
	StereoCalibration* ctrl_;

	nanogui::TabWidget* tabs_;
	nanogui::Button* bsave_;
	ftl::gui2::IntrinsicDetails* infol_;
	ftl::gui2::IntrinsicDetails* infor_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


////////////////////////////////////////////////////////////////////////////////
//

StereoCalibrationStart::StereoCalibrationStart(ftl::gui2::Screen *parent, StereoCalibration *ctrl) :
		ftl::gui2::View(parent), ctrl_(ctrl) {

	show_all_ = false;
	window_ = new FixedWindow(parent, std::string("Stereo Calibration"));
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

StereoCalibrationStart::~StereoCalibrationStart() {
	window_->setVisible(false);
	if (parent()->getRefCount() > 0) {
		window_->dispose();
	}
}

void StereoCalibrationStart::update() {
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

void StereoCalibrationStart::draw(NVGcontext* ctx) {
	window_->center();
	View::draw(ctx);
}

////////////////////////////////////////////////////////////////////////////////
// Capture Window

void StereoCalibrationView::CaptureWindow::update() {
	ctrl_->setChessboard({width_, height_}, square_size_);
}

StereoCalibrationView::CaptureWindow::CaptureWindow(nanogui::Widget* parent, StereoCalibrationView* view) :
	FixedWindow(parent, "Capture Options"), view_(view), ctrl_(view->ctrl_) {

	width_ = ctrl_->chessboardSize().width;
	height_ = ctrl_->chessboardSize().height;
	square_size_ = ctrl_->squareSize();

	setLayout(new nanogui::BoxLayout
		(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 6, 6));

	(new nanogui::Button(buttonPanel(), "", ENTYPO_ICON_CROSS))->setCallback(
		[this]() {
		update();
		view_->setMode(Mode::VIDEO);
	});

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

void StereoCalibrationView::CaptureWindow::draw(NVGcontext* ctx) {
	FixedWindow::draw(ctx);
}

////////////////////////////////////////////////////////////////////////////////
// Control Window

StereoCalibrationView::ControlWindow::ControlWindow(nanogui::Widget* parent, StereoCalibrationView* view) :
	FixedWindow(parent, ""), view_(view), ctrl_(view->ctrl_) {

	setLayout(new nanogui::BoxLayout
		(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 6, 6));

	txtnframes_ = new nanogui::Label(this, "");
	updateCount();

	auto* buttons = new nanogui::Widget(this);
	buttons->setLayout(new nanogui::BoxLayout
		(nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 0, 0));

	auto* button_back = new nanogui::Button(buttons, "", ENTYPO_ICON_ARROW_LEFT);
	button_back->setCallback([this, button = button_back](){
		view_->setMode(Mode::CAPTURE_INIT);
	});

	bresults_ = new nanogui::Button(buttons, "Details");
	bresults_->setFixedWidth(120);
	//bresults_->setEnabled(ctrl_->calib().calibrated());
	bresults_->setCallback([view = view_, button = bresults_]{
		view->setMode(Mode::RESULTS);
	});

	bpause_ = new nanogui::Button(buttons, "");
	bpause_->setFixedWidth(120);
	bpause_->setCallback([&ctrl = ctrl_](){
		ctrl->setCapture(!ctrl->capturing());
	});

	bcalibrate_ = new nanogui::Button(buttons, "Calibrate");
	bcalibrate_->setFixedWidth(120);
	bcalibrate_->setCallback([view = view_, button = bcalibrate_](){
		view->setMode(Mode::CALIBRATION);
	});
}

void StereoCalibrationView::ControlWindow::draw(NVGcontext* ctx) {
	if (ctrl_->capturing())	{ bpause_->setCaption("Pause"); }
	else 					{ bpause_->setCaption("Continue"); }
	//bcalibrate_->setEnabled(ctrl_->calib().count() > 0);
	//bresults_->setEnabled(ctrl_->calib().calibrated());
	updateCount();
	FixedWindow::draw(ctx);
}

void StereoCalibrationView::ControlWindow::updateCount() {
	txtnframes_->setCaption("Detected patterns: " +
							std::to_string(ctrl_->count()));
}
////////////////////////////////////////////////////////////////////////////////
// Calibration Window

StereoCalibrationView::CalibrationWindow::CalibrationWindow(nanogui::Widget* parent, StereoCalibrationView* view) :
		FixedWindow(parent, "Calibration"), view_(view), ctrl_(view->ctrl_) {

	calibrating_ = false;

	setLayout(new nanogui::BoxLayout
		(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 6, 6));

	(new nanogui::Button(buttonPanel(), "", ENTYPO_ICON_CROSS))->setCallback(
		[view = view_]() {
		view->setMode(Mode::VIDEO);
	});

	nanogui::GridLayout *grid_layout = new nanogui::GridLayout
		(nanogui::Orientation::Horizontal, 2, nanogui::Alignment::Fill, 0, 5);
	grid_layout->setColAlignment
		({nanogui::Alignment::Maximum, nanogui::Alignment::Fill});

	grid_layout->setSpacing(0, 10);
	auto* sensor = new nanogui::Widget(this);
	sensor->setLayout(grid_layout);

	// flags
	new nanogui::Label(this, "Flags");
	new ftl::gui2::OpenCVFlagWidget(this, &(ctrl_->flags()));

	bcalibrate_ = new nanogui::Button(this, "Run");
	bcalibrate_->setEnabled(false);
	bcalibrate_->setCallback([&ctrl = ctrl_, &running = calibrating_](){
		if (!ctrl->isBusy()) {
			ctrl->run();
			running = true;
		}
	});
}

void StereoCalibrationView::CalibrationWindow::draw(NVGcontext* ctx) {
	bcalibrate_->setEnabled(!ctrl_->isBusy() && (ctrl_->count() > 0));
	if (calibrating_ && !ctrl_->isBusy()) {
		calibrating_ = false;
		view_->setMode(Mode::RESULTS);
	}
	FixedWindow::draw(ctx);
}

////////////////////////////////////////////////////////////////////////////////
// Result window

StereoCalibrationView::ResultWindow::ResultWindow(nanogui::Widget* parent, StereoCalibrationView* view) :
	FixedWindow(parent, "Results"), view_(view), ctrl_(view->ctrl_) {

	setLayout(new nanogui::BoxLayout
		(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 8 , 0));

	tabs_ = new nanogui::TabWidget(this);
	auto* tabl = tabs_->createTab("Left (intrinsic)");
	auto* tabr = tabs_->createTab("Right (intrinsic)");
	infol_ = new ftl::gui2::IntrinsicDetails(tabl);
	infor_ = new ftl::gui2::IntrinsicDetails(tabr);

	(new nanogui::Button(buttonPanel(), "", ENTYPO_ICON_CROSS))->setCallback(
		[view = view_]() {
		view->setMode(Mode::VIDEO);
	});

	bsave_ = new nanogui::Button(this, "Save");
	bsave_->setCallback([button = bsave_, ctrl = ctrl_](){
		ctrl->saveCalibration();
		button->setCaption("Saved");
		button->setEnabled(false);
	});
}

void StereoCalibrationView::ResultWindow::draw(NVGcontext* ctx) {
	nanogui::Window::draw(ctx);
}

void StereoCalibrationView::ResultWindow::performLayout(NVGcontext* ctx) {
	nanogui::Window::performLayout(ctx);
	auto sz = infor_->preferredSize(ctx);
	infol_->parent()->setSize(sz);
	infor_->parent()->setSize(sz);
	center();
}

void StereoCalibrationView::ResultWindow::update() {
	infol_->update(ctrl_->calibrationLeft().intrinsic);
	infor_->update(ctrl_->calibrationRight().intrinsic);

	bsave_->setEnabled(true);
	bsave_->setCaption("Save");
	screen()->performLayout();
}

////////////////////////////////////////////////////////////////////////////////

StereoCalibrationView::StereoCalibrationView(Screen* parent,
		StereoCalibration* ctrl) : View(parent), ctrl_(ctrl) {

	imview_ = new ftl::gui2::StereoImageView(this);

	int w = 300;
	wcapture_ = new CaptureWindow(screen(), this);
	wcapture_->setFixedWidth(w);
	wcontrol_ = new ControlWindow(screen(), this);
	wcalibration_ = new CalibrationWindow(screen(), this);
	wcalibration_->setFixedWidth(w);
	wresults_ = new ResultWindow(screen(), this);

	screen()->performLayout();
	setMode(Mode::CAPTURE_INIT);
}

StereoCalibrationView::~StereoCalibrationView() {
	wcapture_->setVisible(false);
	wcapture_->dispose();
	wcontrol_->setVisible(false);
	wcontrol_->dispose();
	wcalibration_->setVisible(false);
	wcalibration_->dispose();
	wresults_->setVisible(false);
	wresults_->dispose();
}

void StereoCalibrationView::performLayout(NVGcontext *ctx) {
	auto sz = wcontrol_->size();
	wcontrol_->setPosition(
		nanogui::Vector2i(width() / 2 - sz[0]/2, height() - 30 - sz[1]));

	wcapture_->center();
	wcalibration_->center();
	wresults_->center();

	imview_->setFixedSize(size());

	View::performLayout(ctx);
}

void StereoCalibrationView::draw(NVGcontext *ctx) {
	if (ctrl_->hasFrame()) {
		auto l = ctrl_->getLeft();
		auto r = ctrl_->getRight();

		if (l.size() != cv::Size(0, 0) && r.size() != cv::Size(0, 0)) {
			imview_->left()->copyFrom(l);
			imview_->right()->copyFrom(r);
		}
	}
	View::draw(ctx);
	auto points = ctrl_->previousPoints();
	if (points.size() == 2) {
		drawChessboardCorners(ctx, imview_->left(), points[0]);
		drawChessboardCorners(ctx, imview_->right(), points[1]);
	}
}

void StereoCalibrationView::setMode(Mode m) {
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
}
