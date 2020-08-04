
#include <loguru.hpp>

#include "calibration.hpp"
#include "../../screen.hpp"
#include "../../widgets/popupbutton.hpp"
#include "../../views/calibration/intrinsicview.hpp"
#include "../../views/calibration/extrinsicview.hpp"
#include "../../views/calibration/stereoview.hpp"

#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <ftl/calibration/optimize.hpp>
#include <ftl/calibration/structures.hpp>
#include <ftl/threads.hpp>

#include <nanogui/entypo.h>
#include <nanogui/layout.h>

using ftl::gui2::Calibration;

using ftl::calibration::CalibrationData;
using ftl::codecs::Channel;
using ftl::data::FrameID;
using ftl::data::FrameSetPtr;


// ==== OpenCVCalibrateFlags ===================================================

using ftl::gui2::OpenCVCalibrateFlags;
using ftl::gui2::OpenCVCalibrateFlagsStereo;

int OpenCVCalibrateFlags::defaultFlags() const {
	// For finding distortion coefficients fix focal length and principal point.
	// Otherwise results might be unreliable.
	return (cv::CALIB_FIX_FOCAL_LENGTH |
			cv::CALIB_FIX_PRINCIPAL_POINT |
			cv::CALIB_FIX_ASPECT_RATIO);
}

std::vector<int> OpenCVCalibrateFlags::list() const {
	return {
		cv::CALIB_USE_INTRINSIC_GUESS,
		cv::CALIB_FIX_FOCAL_LENGTH,
		cv::CALIB_FIX_PRINCIPAL_POINT,
		cv::CALIB_FIX_ASPECT_RATIO,
		cv::CALIB_ZERO_TANGENT_DIST,
		cv::CALIB_FIX_K1,
		cv::CALIB_FIX_K2,
		cv::CALIB_FIX_K3,
		cv::CALIB_FIX_K4,
		cv::CALIB_FIX_K5,
		cv::CALIB_FIX_K6,
		cv::CALIB_RATIONAL_MODEL,
		cv::CALIB_THIN_PRISM_MODEL,
		cv::CALIB_FIX_S1_S2_S3_S4,
		cv::CALIB_TILTED_MODEL,
		cv::CALIB_FIX_TAUX_TAUY
	};
}

std::string OpenCVCalibrateFlags::name(int i) const {
	using namespace cv;
	switch(i) {
		case CALIB_FIX_INTRINSIC:
			return "CALIB_FIX_INTRINSIC";

		case CALIB_FIX_FOCAL_LENGTH:
			return "CALIB_FIX_FOCAL_LENGTH";

		case CALIB_USE_INTRINSIC_GUESS:
			return "CALIB_USE_INTRINSIC_GUESS";

		case CALIB_USE_EXTRINSIC_GUESS:
			return "CALIB_USE_EXTRINSIC_GUESS";

		case CALIB_FIX_PRINCIPAL_POINT:
			return "CALIB_FIX_PRINCIPAL_POINT";

		case CALIB_FIX_ASPECT_RATIO:
			return "CALIB_FIX_ASPECT_RATIO";

		case CALIB_SAME_FOCAL_LENGTH:
			return "CALIB_SAME_FOCAL_LENGTH";

		case CALIB_ZERO_TANGENT_DIST:
			return "CALIB_ZERO_TANGENT_DIST";

		case CALIB_FIX_K1:
			return "CALIB_FIX_K1";

		case CALIB_FIX_K2:
			return "CALIB_FIX_K2";

		case CALIB_FIX_K3:
			return "CALIB_FIX_K3";

		case CALIB_FIX_K4:
			return "CALIB_FIX_K4";

		case CALIB_FIX_K5:
			return "CALIB_FIX_K5";

		case CALIB_FIX_K6:
			return "CALIB_FIX_K6";

		case CALIB_RATIONAL_MODEL:
			return "CALIB_RATIONAL_MODEL";

		case CALIB_THIN_PRISM_MODEL:
			return "CALIB_THIN_PRISM_MODEL";

		case CALIB_FIX_S1_S2_S3_S4:
			return "CALIB_FIX_S1_S2_S3_S4";

		case CALIB_TILTED_MODEL:
			return "CALIB_TILTED_MODEL";

		case CALIB_FIX_TAUX_TAUY:
			return "CALIB_FIX_TAUX_TAUY";
	};
	return "";
}


std::string OpenCVCalibrateFlags::explain(int i) const {
	using namespace cv;
	switch(i) {
		case CALIB_FIX_INTRINSIC:
			return "Fix all intrinsic paramters.";

		case CALIB_FIX_FOCAL_LENGTH:
			return "Fix focal length (fx and fy).";

		case CALIB_USE_INTRINSIC_GUESS:
			return "Use valid initial values of fx, fy, cx, cy that are "
					"optimized further. Otherwise, (cx, cy) is initially set "
					"to the image center and focal distances are computed in "
					"a least-squares fashion.";

		case CALIB_USE_EXTRINSIC_GUESS:
			return "";

		case CALIB_FIX_PRINCIPAL_POINT:
			return "The principal point is not changed during the global "
					"optimization. It stays at the center or at a location "
					"specified in initial parameters.";

		case CALIB_FIX_ASPECT_RATIO:
			return "Consider only fy as a free parameter. The ratio fx/fy "
					"stays the same. When CALIB_USE_INTRINSIC_GUESS is not "
					"set, the actual input values of fx and fy are ignored, "
					"only their ratio is computed and used further.";

		case CALIB_ZERO_TANGENT_DIST:
			return "Tangential distortion coefficients (p1,p2) are set to "
					"zeros and stay zero.";

		case CALIB_FIX_K1:
		case CALIB_FIX_K2:
		case CALIB_FIX_K3:
		case CALIB_FIX_K4:
		case CALIB_FIX_K5:
		case CALIB_FIX_K6:
			return "The radial distortion coefficient is not changed during "
					"the optimization. If CALIB_USE_INTRINSIC_GUESS is set, "
					"the coefficient from initial values is used. Otherwise, "
					"it is set to 0.";

		case CALIB_RATIONAL_MODEL:
			return "Coefficients k4, k5, and k6 are enabled.";

		case CALIB_THIN_PRISM_MODEL:
			return " Coefficients s1, s2, s3 and s4 are enabled.";

		case CALIB_FIX_S1_S2_S3_S4:
			return "The thin prism distortion coefficients are not changed "
					"during the optimization. If CALIB_USE_INTRINSIC_GUESS is "
					"set, the supplied coefficients are used. Otherwise, they "
					"are set to 0.";

		case CALIB_TILTED_MODEL:
			return "Coefficients tauX and tauY are enabled";

		case CALIB_FIX_TAUX_TAUY:
			return "The coefficients of the tilted sensor model are not "
					"changed during the optimization. If "
					"CALIB_USE_INTRINSIC_GUESS is set, the supplied "
					"coefficients are used. Otherwise, they are set to 0.";
	};
	return "";
}

std::vector<int> OpenCVCalibrateFlagsStereo::list() const {
	auto ls = OpenCVCalibrateFlags::list();
	ls.insert(ls.begin(), cv::CALIB_FIX_INTRINSIC);
	ls.insert(ls.begin() + 1, cv::CALIB_SAME_FOCAL_LENGTH);
	ls.insert(ls.begin() + 1, cv::CALIB_USE_EXTRINSIC_GUESS);
	return ls;
}


std::string OpenCVCalibrateFlagsStereo::explain(int i) const {
	using namespace cv;
	switch(i) {
		case CALIB_FIX_INTRINSIC:
			return "Fix intrinsic camera paramters (focal length, aspect "
					"ratio, principal point and distortion coefficients)";

		case CALIB_USE_INTRINSIC_GUESS:
			return "Optimize some or all of the intrinsic parameters according "
					"to the specified flags";

		case CALIB_USE_EXTRINSIC_GUESS:
			return "Rotation and translation have valid initial values that "
					"are optimized further. Otherwise rotation and translation "
					"are initialized to the median value of the pattern views ";

		case CALIB_SAME_FOCAL_LENGTH:
			return "Enforce fx_l == fx_r && fy_l == fy_r";

		default:
			return OpenCVCalibrateFlags::explain(i);
	};
}

int OpenCVCalibrateFlagsStereo::defaultFlags() const {
	return cv::CALIB_FIX_INTRINSIC;
}

// ==== Calibration module =====================================================
// Loads sub-modules and adds buttons to main screen.

void Calibration::init() {

	screen->addModule<IntrinsicCalibration>("calib_intrinsic", this, screen, io);
	screen->addModule<ExtrinsicCalibration>("calib_extrinsic", this, screen, io);
	screen->addModule<StereoCalibration>("calib_stereo", this, screen, io);

	// NOTE: If more GUI code is added, consider moving the GUI cude to a new
	//       file in ../views/

	// Should implement PopupMenu widget which would abstract building steps
	// and provide common feel&look. (TODO)

	auto button = screen->addButton<ftl::gui2::PopupButton>("", ENTYPO_ICON_CAMERA);
	button->setChevronIcon(0);
	button->setTooltip("Calibrate Cameras");

	auto* popup = button->popup();
	popup->setLayout(new nanogui::BoxLayout
		(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 10, 6));

	auto* button_intrinsic = new nanogui::Button(popup, "Intrinsic Calibration");
	button_intrinsic->setCallback([this, button, button_intrinsic, popup](){
		button->setPushed(false);
		button_intrinsic->setPushed(false);
		button_intrinsic->setFocused(false);
		auto* calib = screen->getModule<IntrinsicCalibration>();
		auto* view = new ftl::gui2::IntrinsicCalibrationStart(screen, calib);
		screen->setView(view);
	});

	auto* button_extrinsic = new nanogui::Button(popup, "Extrinsic Calibration");
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
	});

	button->setVisible(true);
}

Calibration::~Calibration() {
	// remove button
}

// ==== CalibrationModule ======================================================

using ftl::gui2::CalibrationModule;


bool CalibrationModule::checkFrame(ftl::data::Frame& frame) {

	if (wait_update_) {
		return false;
	}

	if (frame.hasChannel(Channel::CalibrationData)) {

		if (calibration_enabled_ != calibrationEnabled(frame)) {

			LOG(INFO) << std::string(calibration_enabled_ ? "Enabling" : "Disabling") +
						 " calibration (changed outside)";

			setCalibrationMode(frame, calibration_enabled_);
			return false;
		}
	}
	else {
		static bool logged_once__ = false;
		if (!logged_once__) {
			LOG(WARNING) << "No CalibrationData channel, is this a valid camera?";
			logged_once__ = true;
		}
		return false;
	}

	return true;
}

bool CalibrationModule::calibrationEnabled(ftl::data::Frame& frame) {
	auto& calib_data = frame.get<CalibrationData>(Channel::CalibrationData);
	return calib_data.enabled;
}

void CalibrationModule::setCalibration(ftl::data::Frame& frame, CalibrationData data) {
	// previous callbacks are cancelled!
	wait_update_ = true;

	// updates enabled_ status with given calibration data

	auto response = frame.response();
	response.create<CalibrationData>(Channel::CalibrationData) = data;
	update_handle_ = frame.onChange(Channel::CalibrationData,
			[&wait_update = wait_update_,
			 &enabled = calibration_enabled_,
			 value = data.enabled]
			(ftl::data::Frame& frame, ftl::codecs::Channel){

		enabled = value;
		wait_update = false;
		return true;
	});
}

void CalibrationModule::setCalibrationMode(ftl::data::Frame& frame, bool value) {

	if (!frame.hasChannel(Channel::CalibrationData)) {
		LOG(ERROR) << 	"Trying to change calibration status of frame which does "
						"not contain CalibrationData";
		return;
	}

	auto data = CalibrationData(frame.get<CalibrationData>(Channel::CalibrationData));
	data.enabled = value;
	setCalibration(frame, data);
}

void CalibrationModule::setCalibrationMode(bool value) {
	calibration_enabled_ = value;
}

cv::Mat CalibrationModule::getMat(ftl::rgbd::Frame& frame, Channel c) {
	auto& vframe = frame.get<ftl::rgbd::VideoFrame>(c);
	cv::Mat host;
	if (vframe.isGPU())	{ vframe.getGPU().download(host); }
	else					{ host =  vframe.getCPU(); }
	return host;
}

cv::cuda::GpuMat CalibrationModule::getGpuMat(ftl::rgbd::Frame& frame, Channel c) {
	auto& vframe = frame.get<ftl::rgbd::VideoFrame>(c);
	cv::cuda::GpuMat gpu;
	if (!vframe.isGPU())	{ gpu.upload(vframe.getCPU()); }
	else					{ gpu = vframe.getGPU(); }
	return gpu;
}
