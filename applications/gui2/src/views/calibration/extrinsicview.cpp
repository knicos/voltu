#include "extrinsicview.hpp"
#include "visualization.hpp"
#include "../../screen.hpp"
#include "../../widgets/window.hpp"

#include <nanogui/common.h>
#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <nanogui/button.h>
#include <nanogui/checkbox.h>
#include <nanogui/label.h>
#include <nanogui/formhelper.h>

using ftl::gui2::ExtrinsicCalibrationStart;
using ftl::gui2::ExtrinsicCalibrationView;

using ftl::gui2::FixedWindow;

using ftl::data::FrameID;
using ftl::codecs::Channel;

ExtrinsicCalibrationStart::ExtrinsicCalibrationStart(Screen* widget, ExtrinsicCalibration* ctrl) :
		ftl::gui2::View(widget), ctrl_(ctrl), fsid_(-1), sources_(0), show_all_(false) {

	show_all_ = false;
	window_ = new nanogui::Window(screen(), std::string("Extrinsic Calibration"));
	window_->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Vertical,
									 nanogui::Alignment::Fill, 6, 12));

	auto* button_refresh = new nanogui::Button(window_->buttonPanel(), "", ENTYPO_ICON_CCW);
	button_refresh->setCallback([this](){
		update();
		updateSources();
		screen()->performLayout();
	});

	lsframesets_ = new nanogui::Widget(window_);
	lsframesets_->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Vertical,
									 nanogui::Alignment::Fill, 0, 8));

	lselect_ = new nanogui::Label(window_, "Select Cameras");
	lselect_->setVisible(false);

	lssources_ = new nanogui::Widget(window_);
	lssources_->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Vertical,
									 nanogui::Alignment::Fill, 0, 8));

	cball_ = new nanogui::CheckBox(window_, "Show all sources",
		[this](bool v){
			show_all_ = v;
			updateSources();
			screen()->performLayout();
	});
	cball_->setChecked(show_all_);
	cball_->setVisible(false);

	bcontinue_ = new nanogui::Button(window_, "Continue");
	bcontinue_->setEnabled(false);
	bcontinue_->setVisible(false);
	bcontinue_->setCallback([this](){
		ctrl_->start(fsid_, getSources());
	});

	window_->setFixedWidth(400);
	window_->setVisible(true);

	update();
}

ExtrinsicCalibrationStart::~ExtrinsicCalibrationStart() {
	window_->setVisible(false);
	if (parent()->getRefCount() > 0) {
		window_->dispose();
	}
}

void ExtrinsicCalibrationStart::draw(NVGcontext* ctx) {
	window_->center();
	bcontinue_->setEnabled(sources_ != 0);
	ftl::gui2::View::draw(ctx);
}

void ExtrinsicCalibrationStart::resetSources() {
	sources_ = ~uint64_t(0);
}

bool ExtrinsicCalibrationStart::sourceSelected(unsigned int idx) {
	return (sources_ & (uint64_t(1) << idx));
}


void ExtrinsicCalibrationStart::addSource(unsigned int idx) {
	sources_ |= (uint64_t(1) << idx);
}

void ExtrinsicCalibrationStart::removeSource(unsigned int idx) {
	sources_ &= ~(uint64_t(1) << idx);
}

std::vector<FrameID> ExtrinsicCalibrationStart::getSources() {
	std::vector<FrameID> sources;
	unsigned int nmax = ctrl_->listSources(fsid_, show_all_).size();
	CHECK(nmax < 64);

	for (unsigned int i = 0; i < nmax; i++) {
		if (sourceSelected(i)) {
			sources.push_back(FrameID(fsid_, i));
		}
	}
	return sources;
}

void ExtrinsicCalibrationStart::updateSources() {
	while (lssources_->childCount() > 0) {
		lssources_->removeChild(lssources_->childCount() - 1);
	}
	if (fsid_ == (unsigned int)(-1)) {
		return;
	}
	for (const auto& [name, id] : ctrl_->listSources(fsid_, show_all_)) {
		auto* button = new nanogui::Button(lssources_, name);
		button->setFlags(nanogui::Button::Flags::ToggleButton);
		button->setChangeCallback([this, button, id = id.source()](bool value){
			if (value)	{ addSource(id); }
			else		{ removeSource(id); }
		});
		if (sourceSelected(id.source())) {
			button->setPushed(true);
		}
	}
}

void ExtrinsicCalibrationStart::update() {
	while (lsframesets_->childCount() > 0) {
		lsframesets_->removeChild(lsframesets_->childCount() - 1);
	}

	for (const auto& [uri, fsid] : ctrl_->listFrameSets()) {
		auto* button = new nanogui::Button(lsframesets_, uri, ENTYPO_ICON_IMAGES);
		button->setFlags(nanogui::Button::Flags::RadioButton);
		if (fsid == fsid_) { button->setPushed(true); }
		button->setCallback([button, fsid, this](){
			fsid_ = fsid;
			lselect_->setVisible(true);
			cball_->setVisible(true);
			bcontinue_->setVisible(true);
			resetSources();
			updateSources();
			screen()->performLayout();
		});
	}
}

////////////////////////////////////////////////////////////////////////////////

class ExtrinsicCalibrationView::ControlWindow : public FixedWindow {
public:
	ControlWindow(nanogui::Widget* parent, ExtrinsicCalibrationView* view);
	virtual void draw(NVGcontext* ctx) override;

private:
	ExtrinsicCalibrationView* view_;
	ExtrinsicCalibration* ctrl_;


	nanogui::Button* bsave_;
	nanogui::Button* bupload_;
	nanogui::Button* bapply_;
	nanogui::Button* bfreeze_;
	nanogui::Button* bcalibrate_;
	nanogui::Button* bpause_;
	nanogui::Button* bresults_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

ExtrinsicCalibrationView::ControlWindow::ControlWindow(nanogui::Widget* parent, ExtrinsicCalibrationView* view) :
	FixedWindow(parent, ""), view_(view), ctrl_(view->ctrl_) {

	setLayout(new nanogui::BoxLayout
		(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 6, 6));

	auto* buttons = new nanogui::Widget(this);
	buttons->setLayout(new nanogui::BoxLayout
		(nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 0, 0));

	bsave_ = new nanogui::Button(buttons, "", ENTYPO_ICON_SAVE);
	bsave_->setTooltip("Save input to file (Debug)");
	bsave_->setCallback([this](){
		std::string fname = nanogui::file_dialog({{"bin", "Binary file"}}, true);
		ctrl_->saveInput(fname);
	});

	bsave_ = new nanogui::Button(buttons, "", ENTYPO_ICON_FOLDER);
	bsave_->setTooltip("Load input from file (Debug)");
	bsave_->setCallback([this](){
		std::string fname = nanogui::file_dialog({{"bin", "Binary file"}}, true);
		ctrl_->loadInput(fname);
	});

	bupload_ = new nanogui::Button(buttons, "", ENTYPO_ICON_UPLOAD);
	bupload_->setTooltip("Save input to sources");
	bupload_->setCallback([this](){
		ctrl_->updateCalibration();
		bupload_->setTextColor(nanogui::Color(32, 192, 32, 255));
	});

	bapply_ = new nanogui::Button(buttons, "");
	bapply_->setFixedWidth(40);
	bapply_->setTooltip("Rectify stereo images");
	bapply_->setFlags(nanogui::Button::Flags::ToggleButton);
	bapply_->setPushed(view_->rectify());
	bapply_->setChangeCallback([button = bapply_, view = view_](bool v){
		view->setRectify(v);
	});

	bfreeze_ = new nanogui::Button(buttons, "", ENTYPO_ICON_CONTROLLER_PLAY);
	bfreeze_->setFixedWidth(40);
	bfreeze_->setTooltip("Freeze view");
	bfreeze_->setCallback([button=bapply_, view=view_, ctrl=ctrl_](){
		ctrl->setCapture(view->paused());
		view->pause(!view->paused());
	});

	bresults_ = new nanogui::Button(buttons, "Show Calibration");
	//bresults_->setEnabled(ctrl_->calib().calibrated());
	bresults_->setCallback([view = view_, button = bresults_]{
		view->setMode(Mode::RESULTS);
	});

	bpause_ = new nanogui::Button(buttons, "");
	bpause_->setFixedWidth(140);
	bpause_->setCallback([&ctrl = ctrl_, button = bpause_](){
		ctrl->setCapture(!ctrl->capturing());
	});

	bcalibrate_ = new nanogui::Button(buttons, "Calibrate");
	bcalibrate_->setFixedWidth(140);
	bcalibrate_->setCallback([view = view_, button = bcalibrate_](){
		view->setMode(Mode::CALIBRATION);
	});
}

void ExtrinsicCalibrationView::ControlWindow::draw(NVGcontext* ctx) {
	if (ctrl_->capturing())	{
		bpause_->setCaption("Pause");
		view_->setRectify(false);
	}
	else 					{
		bpause_->setCaption("Continue");
	}
	bapply_->setIcon(view_->rectify() ? ENTYPO_ICON_EYE : ENTYPO_ICON_EYE_WITH_LINE);
	bapply_->setPushed(view_->rectify());
	bfreeze_->setIcon(view_->paused() ? ENTYPO_ICON_CONTROLLER_PLAY : ENTYPO_ICON_CONTROLLER_PAUS);
	//bcalibrate_->setEnabled(ctrl_->calib().nFrames() > 0);
	//bresults_->setEnabled(ctrl_->calib().calibrated());
	FixedWindow::draw(ctx);
}

////////////////////////////////////////////////////////////////////////////////

class ExtrinsicCalibrationView::CalibrationWindow : public FixedWindow {
public:
	CalibrationWindow(nanogui::Widget* parent, ExtrinsicCalibrationView* view);
	virtual void draw(NVGcontext* ctx) override;

private:
	void build();

	ExtrinsicCalibrationView* view_;
	ExtrinsicCalibration* ctrl_;
	nanogui::Widget* cameras_;

	nanogui::Label* status_;
	nanogui::Button* brun_;
	bool running_; // run button clicked
	int flags_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

ExtrinsicCalibrationView::CalibrationWindow::CalibrationWindow(nanogui::Widget* parent, ExtrinsicCalibrationView* view) :
	FixedWindow(parent, "Settings"), view_(view), ctrl_(view->ctrl_) {

	running_ = false;

	(new nanogui::Button(buttonPanel(), "", ENTYPO_ICON_CROSS))->setCallback(
	[view = view_]() {
		view->setMode(Mode::VIDEO);
	});

	setLayout(new nanogui::BoxLayout
		(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 10 , 10));

	build();
}

void ExtrinsicCalibrationView::CalibrationWindow::build() {

	flags_ = ctrl_->flags();

	auto* wfreeze = new nanogui::Widget(this);
	wfreeze->setLayout(new nanogui::BoxLayout
		(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 0 , 5));

	auto* floss = new nanogui::CheckBox(wfreeze, "Cauchy loss");
	floss->setChecked(flags_ & ExtrinsicCalibration::Flags::LOSS_CAUCHY);
	floss->setCallback([&flags = flags_](bool v) {
		if (v)	{ flags |= ExtrinsicCalibration::Flags::LOSS_CAUCHY; }
		else	{ flags &= ~ExtrinsicCalibration::Flags::LOSS_CAUCHY; }
	});

	auto* nstep = new nanogui::CheckBox(wfreeze, "Non-monotonic step");
	nstep->setChecked(flags_ & ExtrinsicCalibration::Flags::NONMONOTONIC_STEP);
	nstep->setCallback([&flags = flags_](bool v) {
		if (v)	{ flags |= ExtrinsicCalibration::Flags::NONMONOTONIC_STEP; }
		else	{ flags &= ~ExtrinsicCalibration::Flags::NONMONOTONIC_STEP; }
	});

	auto* fall = new nanogui::CheckBox(wfreeze, "Freeze all intrinsic paramters");
	fall->setChecked(flags_ & ExtrinsicCalibration::Flags::FIX_INTRINSIC);
	fall->setCallback([&flags = flags_, wfreeze](bool v) {
		for (int i = 3; i < wfreeze->childCount(); i++) {
			wfreeze->childAt(i)->setEnabled(!v);
		}
		if (v)	{ flags |= ExtrinsicCalibration::Flags::FIX_INTRINSIC; }
		else	{ flags &= ~ExtrinsicCalibration::Flags::FIX_INTRINSIC; }
	});

	auto* ff = new nanogui::CheckBox(wfreeze, "Fix focal length");
	ff->setChecked(flags_ & ExtrinsicCalibration::Flags::FIX_FOCAL);
	ff->setCallback([&flags = flags_](bool v) {
		if (v)	{ flags |= ExtrinsicCalibration::Flags::FIX_FOCAL; }
		else	{ flags &= ~ExtrinsicCalibration::Flags::FIX_FOCAL; }
	});

	auto* fpp = new nanogui::CheckBox(wfreeze, "Fix principal point");
	fpp->setChecked(flags_ & ExtrinsicCalibration::Flags::FIX_PRINCIPAL_POINT);
	fpp->setCallback([&flags = flags_](bool v) {
		if (v)	{ flags |= ExtrinsicCalibration::Flags::FIX_PRINCIPAL_POINT; }
		else	{ flags &= ~ExtrinsicCalibration::Flags::FIX_PRINCIPAL_POINT; }
	});

	auto* fdist = new nanogui::CheckBox(wfreeze, "Fix distortion coefficients");
	fdist->setChecked(flags_ & ExtrinsicCalibration::Flags::FIX_DISTORTION);
	fdist->setCallback([&flags = flags_](bool v) {
		if (v)	{ flags |= ExtrinsicCalibration::Flags::FIX_DISTORTION; }
		else	{ flags &= ~ExtrinsicCalibration::Flags::FIX_DISTORTION; }
	});

	auto* zdist = new nanogui::CheckBox(wfreeze, "Assume zero distortion");
	zdist->setChecked(flags_ & ExtrinsicCalibration::Flags::ZERO_DISTORTION);
	zdist->setCallback([&flags = flags_](bool v) {
		if (v)	{ flags |= ExtrinsicCalibration::Flags::ZERO_DISTORTION; }
		else	{ flags &= ~ExtrinsicCalibration::Flags::ZERO_DISTORTION; }
	});

	auto* rdist = new nanogui::CheckBox(wfreeze, "Rational distortion model");
	rdist->setChecked(flags_ & ExtrinsicCalibration::Flags::RATIONAL_MODEL);
	rdist->setCallback([&flags = flags_](bool v) {
		if (v)	{ flags |= ExtrinsicCalibration::Flags::RATIONAL_MODEL; }
		else	{ flags &= ~ExtrinsicCalibration::Flags::RATIONAL_MODEL; }
	});

	/* Needs thinking: visualize visibility graph? Use earlier alignment (if
	 * some of the cameras already calibrated), do elsewhere?
	 */

	status_ = new nanogui::Label(this, "Ready");
	brun_ = new nanogui::Button(this, "Run");
	brun_->setCallback([this](){
		ctrl_->setFlags(flags_);
		ctrl_->run();
		running_ = true;
	});
}

void ExtrinsicCalibrationView::CalibrationWindow::draw(NVGcontext* ctx) {
	brun_->setEnabled(!ctrl_->isBusy());
	if (ctrl_->isBusy()) {
		if (running_) {
			auto dots = std::string(int(round(glfwGetTime())) % 4, '.');
			status_->setCaption(ctrl_->status() + dots);
		}
		else {
			status_->setCaption("Busy");
		}
	}
	else {
		status_->setCaption("Ready");
	}
	if (running_ && !ctrl_->isBusy()) {
		running_ = false;
		view_->setMode(Mode::RESULTS);
	}
	FixedWindow::draw(ctx);
}

////////////////////////////////////////////////////////////////////////////////

class ExtrinsicCalibrationView::ResultsWindow : public FixedWindow {
public:
	ResultsWindow(nanogui::Widget* parent, ExtrinsicCalibrationView* view);
	virtual void draw(NVGcontext* ctx) override;

private:
	ExtrinsicCalibrationView* view_;
	ExtrinsicCalibration* ctrl_;

	nanogui::Button* bcalibrate_;
	nanogui::Button* bpause_;
	nanogui::Button* bresults_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

ExtrinsicCalibrationView::ResultsWindow::ResultsWindow(nanogui::Widget* parent, ExtrinsicCalibrationView* view) :
	FixedWindow(parent, "Results"), view_(view), ctrl_(view->ctrl_) {

	(new nanogui::Button(buttonPanel(), "", ENTYPO_ICON_CROSS))->setCallback(
		[view = view_]() {
		view->setMode(Mode::VIDEO);
	});
	setSize({300, 300});
}

void ExtrinsicCalibrationView::ResultsWindow::draw(NVGcontext* ctx) {
	FixedWindow::draw(ctx);
	std::vector<ftl::calibration::CalibrationData::Extrinsic> calib(ctrl_->cameraCount());
	for (int i = 0; i < ctrl_->cameraCount(); i++) {
		calib[i] = ctrl_->calibration(i).extrinsic;
	}
	drawFloorPlan(ctx, this, calib);
}

////////////////////////////////////////////////////////////////////////////////

static void drawText(NVGcontext* ctx, nanogui::Vector2f &pos, const std::string& text,
		float size=12.0f, int align=NVG_ALIGN_MIDDLE|NVG_ALIGN_CENTER) {
	nvgFontSize(ctx, size);
	nvgFontFace(ctx, "sans-bold");
	nvgTextAlign(ctx, align);
	nvgFillColor(ctx, nanogui::Color(8, 8, 8, 255)); // shadow
	nvgText(ctx, pos.x(), pos.y(), text.c_str(), nullptr);
	nvgFillColor(ctx, nanogui::Color(244, 244, 244, 255));
	nvgText(ctx, pos.x() + 1, pos.y() + 1, text.c_str(), nullptr);
}

ExtrinsicCalibrationView::ExtrinsicCalibrationView(Screen* widget, ExtrinsicCalibration* ctrl) :
		ftl::gui2::View(widget), ctrl_(ctrl), rows_(0) {

	frames_ = new nanogui::Widget(this);
	draw_number_ = false;
	rectify_ = false;

	frames_->setLayout(new nanogui::BoxLayout
		(nanogui::Orientation::Horizontal, nanogui::Alignment::Maximum, 0, 0));

	// assumes all cameras stereo cameras, indexed in order
	for (int i = 0; i < ctrl_->cameraCount(); i += 2) {
		new StereoImageView(frames_, nanogui::Orientation::Vertical);
	}
	paused_ = false;
	wcontrol_ = new ControlWindow(screen(), this);
	wcalibration_ = new CalibrationWindow(screen(), this);
	wresults_ = new ResultsWindow(screen(), this);
	setMode(Mode::CAPTURE_IMAGES);
}

void ExtrinsicCalibrationView::performLayout(NVGcontext* ctx) {

	auto sz = wcontrol_->size();
	wcontrol_->setPosition(
		nanogui::Vector2i(width() / 2 - sz[0]/2, height() - 30 - sz[1]));

	wcalibration_->center();
	wresults_->center();

	frames_->setSize(size());

	nanogui::Vector2i fsize = { width()/(frames_->childCount()), height() };
	for (int i = 0; i < frames_->childCount(); i++) {
		auto* stereo = dynamic_cast<StereoImageView*>(frames_->childAt(i));
		stereo->setFixedSize(fsize);
		stereo->fit();
	}

	View::performLayout(ctx);
}

void ExtrinsicCalibrationView::draw(NVGcontext* ctx) {

	if (ctrl_->next() && !paused_) {
		for (int i = 0; i < ctrl_->cameraCount(); i += 2) {
			auto* imview = dynamic_cast<StereoImageView*>(frames_->childAt(i/2));

			int l = i;
			int r = i + 1;
			if (ctrl_->hasFrame(l)) {
				if (!rectify_) { imview->left()->copyFrom(ctrl_->getFrame(l)); }
				else { imview->left()->copyFrom(ctrl_->getFrameRectified(l)); }
				imview->left()->setVisible(true);
			}
			else { imview->left()->setVisible(false); }

			if (ctrl_->hasFrame(r)) {
				if (!rectify_) { imview->right()->copyFrom(ctrl_->getFrame(r)); }
				else { imview->right()->copyFrom(ctrl_->getFrameRectified(r)); }
				imview->right()->setVisible(true);
			}
			else { imview->right()->setVisible(false); }
		}
	}

	Widget::draw(ctx);

	// draw corner labels
	for (int i = 0; i < ctrl_->cameraCount(); i++) {
		FTLImageView* imview;
		if (i%2 == 0) {
			imview = dynamic_cast<StereoImageView*>(frames_->childAt(i/2))->left();
		}
		else {
			imview = dynamic_cast<StereoImageView*>(frames_->childAt(i/2))->right();
		}
		auto points = ctrl_->previousPoints(i);

		std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>
			paths;


		nanogui::Vector2f wpos = imview->absolutePosition().cast<float>();
		nanogui::Vector2f wsize = imview->sizeF();

		for (unsigned int p = 0; p < points.size(); p++) {
			auto pos = imview->positionForCoordinate({points[p].x, points[p].y});
			nanogui::Vector2f apos = pos + wpos;
			paths.push_back(apos);
		}

		nvgScissor(ctx, wpos.x(), wpos.y(), wsize.x(), wsize.y());
		// draw border
		for (unsigned int p = 0; p < paths.size(); p += 4) {
			nvgBeginPath(ctx);
			nvgMoveTo(ctx, paths[p + 0].x(), paths[p + 0].y());
			nvgLineTo(ctx, paths[p + 1].x(), paths[p + 1].y());
			nvgLineTo(ctx, paths[p + 2].x(), paths[p + 2].y());
			nvgLineTo(ctx, paths[p + 3].x(), paths[p + 3].y());
			nvgLineTo(ctx, paths[p + 0].x(), paths[p + 0].y());
			if (p == 0) nvgStrokeColor(ctx, nvgRGBA(255, 32, 32, 255));
			if (p == 4) nvgStrokeColor(ctx, nvgRGBA(32, 255, 32, 255));
			nvgStrokeWidth(ctx, 1.5f);
			nvgStroke(ctx);
		}
		// draw number
		/*if (draw_number_ ) {
			for (unsigned int p = 0; p < paths.size(); p += 1) {
				auto str = std::to_string(p);
				drawText(ctx, paths[p], std::to_string(p), 14.0f);
			}
		}*/
		nanogui::Vector2f cpos = wpos + nanogui::Vector2f{10.0f, 10.0f};
		drawText(ctx, cpos, std::to_string(ctrl_->getFrameCount(i)), 20.0f, NVG_ALIGN_TOP|NVG_ALIGN_LEFT);
		nvgResetScissor(ctx);
	}
}

ExtrinsicCalibrationView::~ExtrinsicCalibrationView() {
	wcontrol_->dispose();
	wcalibration_->dispose();
	wresults_->dispose();
}

void ExtrinsicCalibrationView::setMode(Mode mode) {
	switch(mode) {
		case Mode::CAPTURE_IMAGES:
			ctrl_->setCapture(true);
			wcontrol_->setVisible(true);
			wcalibration_->setVisible(false);
			wresults_->setVisible(false);
			break;

		case Mode::VIDEO:
			ctrl_->setCapture(false);
			wcontrol_->setVisible(true);
			wcalibration_->setVisible(false);
			wresults_->setVisible(false);
			break;

		case Mode::CALIBRATION:
			ctrl_->setCapture(false);
			wcontrol_->setVisible(false);
			wcalibration_->setVisible(true);
			wresults_->setVisible(false);
			break;

		case Mode::RESULTS:
			ctrl_->setCapture(false);
			wcontrol_->setVisible(false);
			wcalibration_->setVisible(false);
			wresults_->setVisible(true);
			//wresults_->update();
			break;
	}
	screen()->performLayout();
}
