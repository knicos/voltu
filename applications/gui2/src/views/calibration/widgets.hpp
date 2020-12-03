/**
 * @file widgets.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Sebastian Hahta
 */

#pragma once

#include <nanogui/widget.h>

#include <ftl/calibration/structures.hpp>

#include "../../modules/calibration/calibration.hpp"

namespace ftl {
namespace gui2 {

class OpenCVFlagWidget : public nanogui::Widget {
public:
	OpenCVFlagWidget(nanogui::Widget* parent, OpenCVCalibrateFlags* flags, int defaultv=-1);
	void reset();
	void setDefaults(int v) { defaults_ = v; }

private:
	OpenCVCalibrateFlags* flags_;
	int defaults_;
};

class IntrinsicDetails : public nanogui::Widget {
public:
	IntrinsicDetails(nanogui::Widget* parent);
	void update(const ftl::calibration::CalibrationData::Intrinsic &values);

private:
	nanogui::Widget* params_;
	nanogui::Widget* dist_;
	int padding_;
};

}
}
