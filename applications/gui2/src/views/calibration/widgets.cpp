/**
 * @file widgets.cpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Sebastian Hahta
 */

#include "widgets.hpp"

#include <nanogui/label.h>
#include <nanogui/layout.h>
#include <nanogui/checkbox.h>

#include <opencv2/calib3d.hpp>

using ftl::gui2::OpenCVFlagWidget;
using ftl::gui2::OpenCVCalibrateFlags;

template<typename T>
std::string to_string(T v, int precision = 2) {
	std::stringstream stream;
	stream << std::fixed << std::setprecision(precision) << v;
	return stream.str();
}

OpenCVFlagWidget::OpenCVFlagWidget(nanogui::Widget* parent, OpenCVCalibrateFlags* flags, int defaultv) :
		nanogui::Widget(parent), flags_(flags), defaults_(defaultv) {

	if (defaultv == -1) {
		defaults_ = flags_->defaultFlags();
	}

	setLayout(new nanogui::BoxLayout
		(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 0, 4));

	reset();
}

void OpenCVFlagWidget::reset() {
	while(childCount() > 0) {
		removeChild(childCount() - 1);
	}

	for(int flag : flags_->list()) {
		auto* checkbox = new nanogui::CheckBox(this, flags_->name(flag),
		[flag, this](bool state){
			if (state)	{ flags_->set(flag); }
			else		{ flags_->unset(flag); }
		});
		checkbox->setChecked(flags_->has(flag));
		checkbox->setTooltip(flags_->explain(flag));
	}

	// reset button
	auto* reset = new nanogui::Button(this, "Reset flags");
	reset->setCallback([this](){

		// update widget
		auto all_flags = flags_->list();
		for(size_t i = 0; i < all_flags.size(); i++) {
			auto* checkbox = dynamic_cast<nanogui::CheckBox*>(childAt(i));
			checkbox->setChecked(all_flags[i] & defaults_);
		}
	});
}

////////////////////////////////////////////////////////////////////////////////

using ftl::gui2::IntrinsicDetails;

IntrinsicDetails::IntrinsicDetails(nanogui::Widget* parent) :
	nanogui::Widget(parent), padding_(8) {

	setLayout(new nanogui::BoxLayout
		(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 0 , padding_));

	params_ = new nanogui::Widget(this);
	dist_ = new nanogui::Widget(this);
	dist_->setLayout(new nanogui::BoxLayout
		(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 0, padding_));
}

void IntrinsicDetails::update(const ftl::calibration::CalibrationData::Intrinsic &values) {
	while (params_->childCount() > 0) {
		params_->removeChild(params_->childCount() - 1);
	}
	while (dist_->childCount() > 0) {
		dist_->removeChild(dist_->childCount() - 1);
	}
	bool use_physical = values.sensorSize != cv::Size2d{0.0, 0.0};
	nanogui::GridLayout* grid_layout;
	if (use_physical) {
		grid_layout = new nanogui::GridLayout
			(nanogui::Orientation::Horizontal, 3, nanogui::Alignment::Fill, 0, padding_);
	}
	else {
		grid_layout = new nanogui::GridLayout
			(nanogui::Orientation::Horizontal, 2, nanogui::Alignment::Fill, 0, padding_);
	}
	grid_layout->setColAlignment
		({nanogui::Alignment::Maximum, nanogui::Alignment::Fill});
	params_->setLayout(grid_layout);

	auto sw = values.sensorSize.width;
	auto sh = values.sensorSize.height;
	auto K = values.matrix();
	auto imsize = values.resolution;

	double fovx;
	double fovy;
	double f;
	cv::Point2d pp;
	double ar;
	cv::calibrationMatrixValues(K, imsize, sw, sh, fovx, fovy, f, pp, ar);

	new nanogui::Label(params_, "Size (sensor/image):");
	if (use_physical) new nanogui::Label(params_, to_string(sw, 1) + std::string("x") + to_string(sh, 1));
	new nanogui::Label(params_, std::to_string(imsize.width) + std::string("x") + std::to_string(imsize.height));

	new nanogui::Label(params_, "Focal length:");
	if (use_physical) new nanogui::Label(params_, to_string(f) + " mm");
	new nanogui::Label(params_,
		((values.fx == values.fy) ? to_string(values.fx) + " px": (
		"(" + to_string(values.fx) + ", "
			+ to_string(values.fy) + ")")));

	new nanogui::Label(params_, "Principal point:");
	if (use_physical) new nanogui::Label(params_,
			"(" + to_string(pp.x) + ", " +
				to_string(pp.y) + ")");

	new nanogui::Label(params_,
		"(" + to_string(values.cx) + ", " +
			  to_string(values.cy) + ")");

	new nanogui::Widget(params_);
	new nanogui::Label(params_,
			"(" + to_string(100.0*(2.0*values.cx/double(imsize.width) - 1.0)) + "% , " +
				to_string(100.0*(2.0*values.cy/double(imsize.height) - 1.0)) + "%)");
	if (use_physical) new nanogui::Widget(params_);

	new nanogui::Label(params_, "Field of View (x):");
	new nanogui::Label(params_, to_string(fovx) + "°");
	if (use_physical) new nanogui::Widget(params_);

	new nanogui::Label(params_, "Field of View (y):");
	new nanogui::Label(params_, to_string(fovy)+ "°");
	if (use_physical) new nanogui::Widget(params_);

	new nanogui::Label(params_, "Aspect ratio:");
	new nanogui::Label(params_, to_string(ar));
	if (use_physical) new nanogui::Widget(params_);

	std::string pK;
	std::string pP;
	std::string pS;
	auto& D = values.distCoeffs;

	pK += "K1: " + to_string(D[0] ,3);
	pK += ", K2: " + to_string(D[1] ,3);
	pP += "P1: " + to_string(D[2], 3);
	pP += ", P2: " + to_string(D[3], 3);

	pK += ", K3: " + to_string(D[4], 3);

	pK += ", K4: " + to_string(D[5] ,3);
	pK += ", K5: " + to_string(D[6] ,3);
	pK += ", K6: " + to_string(D[7] ,3);

	pS += "S1: " + to_string(D[8] ,3);
	pS += ", S2: " + to_string(D[9] ,3);
	pS += ", S3: " + to_string(D[10] ,3);
	pS += ", S4: " + to_string(D[11] ,3);

	if (!pK.empty()) new nanogui::Label(dist_, pK);
	if (!pP.empty()) new nanogui::Label(dist_, pP);
	if (!pS.empty()) new nanogui::Label(dist_, pS);
}
