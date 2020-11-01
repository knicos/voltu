#include "developer.hpp"
#include <nanogui/entypo.h>
#include <nanogui/layout.h>
#include "../../widgets/popupbutton.hpp"
#include "../../screen.hpp"

using ftl::gui2::Developer;

void Developer::init() {
	//screen->addModule<ExtrinsicCalibration>("calib_extrinsic", this, screen, io);
	//screen->addModule<StereoCalibration>("calib_stereo", this, screen, io);

	// NOTE: If more GUI code is added, consider moving the GUI cude to a new
	//       file in ../views/

	// Should implement PopupMenu widget which would abstract building steps
	// and provide common feel&look. (TODO)

	auto button = screen->addButton<ftl::gui2::PopupButton>("", ENTYPO_ICON_TOOLS);
	button->setChevronIcon(0);
	button->setTooltip("Developer Tools");

	auto* popup = button->popup();
	popup->setLayout(new nanogui::BoxLayout
		(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 10, 6));

	auto* button_disp = new nanogui::Button(popup, "Disparity Tools");
	button_disp->setCallback([this, button, button_disp, popup](){
		button->setPushed(false);
		button_disp->setPushed(false);
		button_disp->setFocused(false);
		auto* disp = screen->getModuleNoExcept<DisparityDev>();
		//auto* view = new ftl::gui2::IntrinsicCalibrationStart(screen, calib);
		//screen->setView(view);

		if (!disp) screen->addModule<DisparityDev>("disparity_dev", this, screen, io);
	});

	/*auto* button_extrinsic = new nanogui::Button(popup, "Extrinsic Calibration");
	button_extrinsic->setCallback([this, button, button_extrinsic, popup](){
		button->setPushed(false);
		button_extrinsic->setPushed(false);
		button_extrinsic->setFocused(false);
		auto* calib = screen->getModule<ExtrinsicCalibration>();
		auto* view = new ftl::gui2::ExtrinsicCalibrationStart(screen, calib);
		screen->setView(view);
	});

	auto* button_stereo = new nanogui::Button(popup, "Stereo Calibration");
	button_stereo->setCallback([this, button, button_extrinsic, popup](){
		button->setPushed(false);
		button_extrinsic->setPushed(false);
		button_extrinsic->setFocused(false);
		auto* calib = screen->getModule<StereoCalibration>();
		auto* view = new ftl::gui2::StereoCalibrationStart(screen, calib);
		screen->setView(view);
	});*/

	button->setVisible(true);
}

Developer::~Developer() {
	// remove button
}