#pragma once

#include "../../modules/calibration/calibration.hpp"
#include "../../view.hpp"
#include "../../widgets/imageview.hpp"

#include <ftl/utility/gltexture.hpp>

namespace ftl
{
namespace gui2
{

class StereoCalibrationStart : public View {
public:
	StereoCalibrationStart(Screen* widget, StereoCalibration* ctrl);
	virtual ~StereoCalibrationStart();

	virtual void draw(NVGcontext *ctx) override;

	void update();

private:
	nanogui::Window* window_;
	nanogui::Widget* buttons_;
	StereoCalibration* ctrl_;
	bool show_all_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class StereoCalibrationView : public View {

	class CaptureWindow;
	class ControlWindow;
	class CalibrationWindow;
	class ResultWindow;

public:
	StereoCalibrationView(Screen* screen, StereoCalibration* ctrl);
	virtual ~StereoCalibrationView();

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

private:
	StereoCalibration* ctrl_;
	StereoImageView* imview_;

	CaptureWindow* wcapture_;
	ControlWindow* wcontrol_;
	CalibrationWindow* wcalibration_;
	ResultWindow* wresults_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace gui2
} // namespace ftl
