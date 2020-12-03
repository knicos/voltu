/**
 * @file intrinsicview.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Sebastian Hahta
 */

#pragma once

#include "../../modules/calibration/calibration.hpp"
#include "../../view.hpp"
#include "../../widgets/imageview.hpp"

#include <ftl/utility/gltexture.hpp>

namespace ftl
{
namespace gui2
{

class IntrinsicCalibrationStart : public View {
public:
	IntrinsicCalibrationStart(Screen* widget, IntrinsicCalibration* ctrl);
	virtual ~IntrinsicCalibrationStart();

	virtual void draw(NVGcontext *ctx) override;

	void update();

private:
	nanogui::Window* window_;
	nanogui::Widget* buttons_;
	IntrinsicCalibration* ctrl_;
	bool show_all_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class IntrinsicCalibrationView : public View {

	class CaptureWindow;
	class ControlWindow;
	class CalibrationWindow;
	class ResultWindow;

public:
	IntrinsicCalibrationView(Screen* screen, IntrinsicCalibration* ctrl);
	virtual ~IntrinsicCalibrationView();

	enum Mode {
		CAPTURE_INIT,	// set capture parameters
		CAPTURE_IMAGES,	// capture images
		CALIBRATION,	// calibration options
		RESULTS,		// calibration results
		VIDEO			// same as capture images but paused
	};

	void setMode(Mode m);

	virtual void performLayout(NVGcontext* ctx) override;
	virtual void draw(NVGcontext* ctx) override;

	void setUndistort(bool v) { undistort_ = v; }
	bool undistort() { return undistort_; }

private:
	IntrinsicCalibration* ctrl_;
	FTLImageView* imview_;

	CaptureWindow* wcapture_;
	ControlWindow* wcontrol_;
	CalibrationWindow* wcalibration_;
	ResultWindow* wresults_;
	bool undistort_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace gui2
} // namespace ftl
