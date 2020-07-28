#pragma once

#include <unordered_set>

#include "../../modules/calibration/calibration.hpp"
#include "../../view.hpp"
#include <ftl/utility/gltexture.hpp>
#include "../../widgets/imageview.hpp"

namespace ftl
{
namespace gui2
{

class ExtrinsicCalibrationStart : public View {
public:
	ExtrinsicCalibrationStart(Screen* widget, ExtrinsicCalibration* ctrl);
	virtual ~ExtrinsicCalibrationStart();

	virtual void draw(NVGcontext *ctx) override;

	/** query about current state */
	void addSource(unsigned int);
	void removeSource(unsigned int);
	void resetSources();
	bool sourceSelected(unsigned int source);
	std::vector<ftl::data::FrameID> getSources();

	/** update widgets */
	void update();
	void updateSources();

private:
	ExtrinsicCalibration* ctrl_;
	nanogui::Window* window_;
	nanogui::Label* lselect_;
	nanogui::CheckBox* cball_;
	nanogui::Widget* lsframesets_;
	nanogui::Widget* lssources_;
	nanogui::Button* bcontinue_;
	unsigned int fsid_;
	uint64_t sources_;
	bool show_all_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class ExtrinsicCalibrationView : public View {
public:
	class ControlWindow;
	class CalibrationWindow;
	class ResultsWindow;

	enum Mode {
		CAPTURE_IMAGES,	// capture images
		CALIBRATION,	// calibration options
		RESULTS,		// calibration results
		VIDEO			// same as capture images but paused
	};

	ExtrinsicCalibrationView(Screen* widget, ExtrinsicCalibration* ctrl);
	virtual ~ExtrinsicCalibrationView();

	virtual void draw(NVGcontext *ctx) override;
	virtual void performLayout(NVGcontext *ctx) override;

	bool rectify() { return rectify_; };
	void setRectify(bool v) { rectify_ = v; };
	void setMode(Mode m);

protected:
	int rows(); // calculate optimum number of rows;
	void setRows(int rows);

private:
	ExtrinsicCalibration* ctrl_;
	nanogui::Widget* frames_;

	ControlWindow* wcontrol_;
	CalibrationWindow* wcalibration_;
	ResultsWindow* wresults_;

	int rows_;
	bool draw_number_;
	bool rectify_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
}
