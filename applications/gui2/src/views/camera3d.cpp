#include "camera3d.hpp"
#include "../modules/camera.hpp"

#include <loguru.hpp>

using ftl::gui2::CameraView3D;

// =============================================================================

static Eigen::Affine3d create_rotation_matrix(float ax, float ay, float az) {
	Eigen::Affine3d rx =
		Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
	Eigen::Affine3d ry =
		Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
	Eigen::Affine3d rz =
		Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
	return rz * rx * ry;
}

// ==== CameraView3D ===========================================================

CameraView3D::CameraView3D(ftl::gui2::Screen *parent, ftl::gui2::Camera *ctrl) :
		CameraView(parent, ctrl) {

	eye_ = Eigen::Vector3d::Zero();
	neye_ = Eigen::Vector4d::Zero();
	rotmat_.setIdentity();

	rx_ = 0.0;
	ry_ = 0.0;

	ftime_ = 0.0;
	delta_ = 0.0;
	lerp_speed_ = 0.999f;

	pose_up_to_date_.test_and_set();

	tools_->setAvailable({
		Tools::SELECT_POINT,
		Tools::MOVEMENT,
		Tools::OVERLAY,
		Tools::INSPECT_POINT,
		Tools::CLIPPING,
		Tools::MOVE_CURSOR,
		Tools::ORIGIN_TO_CURSOR,
		Tools::RESET_ORIGIN,
		Tools::SAVE_CURSOR,
		Tools::ROTATE_X,
		Tools::ROTATE_Y,
		Tools::ROTATE_Z,
		Tools::TRANSLATE_X,
		Tools::TRANSLATE_Y,
		Tools::TRANSLATE_Z
	});

	setZoom(false);
	setPan(false);

	tools_->setTool(Tools::MOVEMENT);

	tools_->addCallback([this](ftl::gui2::Tools tool) {
		if (tool == Tools::ORIGIN_TO_CURSOR) {
			ctrl_->setOriginToCursor();
			tools_->setTool(Tools::MOVEMENT);
			return true;
		} else if (tool == Tools::RESET_ORIGIN) {
			ctrl_->resetOrigin();
			tools_->setTool(Tools::MOVEMENT);
			return true;
		} else if (tool == Tools::SAVE_CURSOR) {
			ctrl_->saveCursorToPoser();
			tools_->setTool(Tools::MOVEMENT);
			return true;
		} else if (tool == Tools::ROTATE_X || tool == Tools::ROTATE_Y || tool == Tools::ROTATE_Z ||
					tool == Tools::TRANSLATE_X || tool == Tools::TRANSLATE_Y || tool == Tools::TRANSLATE_Z) {
			LOG(INFO) << "Loading cache pose";
			cache_pose_ = ctrl_->getActivePose();
			cache_screen_ = ctrl_->getActivePoseScreenCoord();
		}
		return false;
	});
}

bool CameraView3D::keyboardEvent(int key, int scancode, int action, int modifiers) {
	if (key == 263 || key == 262) {
		float mag = (modifiers & 0x1) ? 0.01f : 0.1f;
		float scalar = (key == 263) ? -mag : mag;
		neye_ += rotmat_*Eigen::Vector4d(scalar, 0.0, 0.0, 1.0);
		pose_up_to_date_.clear();
	}
	else if (key == 264 || key == 265) {
		float mag = (modifiers & 0x1) ? 0.01f : 0.1f;
		float scalar = (key == 264) ? -mag : mag;
		neye_ += rotmat_*Eigen::Vector4d(0.0, 0.0, scalar, 1.0);
		pose_up_to_date_.clear();
	}
	else if (key == 266 || key == 267) {
		float mag = (modifiers & 0x1) ? 0.01f : 0.1f;
		float scalar = (key == 266) ? -mag : mag;
		neye_ += rotmat_*Eigen::Vector4d(0.0, scalar, 0.0, 1.0);
		pose_up_to_date_.clear();
	}
	else if (key >= '0' && key <= '5' && modifiers == 2) {  // Ctrl+NUMBER
	}

	return true;
}

bool CameraView3D::mouseButtonEvent(const Eigen::Vector2i &p, int button, bool down, int modifiers) {
	if (button == 0 && !down) {
		if (tools_->isActive(Tools::MOVE_CURSOR)) {
			auto mouse = screen()->mousePos();
			auto pos = imview_->imageCoordinateAt((mouse - mPos).cast<float>());
			//Eigen::Vector3f world = ctrl_->worldAt(pos.x(), pos.y());

			ctrl_->setCursor(pos.x(), pos.y());
			tools_->setTool(Tools::ROTATE_CURSOR);
			return true;
		} else if (tools_->isActive(Tools::ROTATE_CURSOR)) {
			tools_->setTool(Tools::MOVEMENT);
		} else if (tools_->isActive(Tools::ROTATE_X) || tools_->isActive(Tools::ROTATE_Y) || tools_->isActive(Tools::ROTATE_Z) ||
					tools_->isActive(Tools::TRANSLATE_X) || tools_->isActive(Tools::TRANSLATE_Y) || tools_->isActive(Tools::TRANSLATE_Z)) {
			tools_->setTool(Tools::MOVEMENT);
		}
	}

	return CameraView::mouseButtonEvent(p, button, down, modifiers);
}

bool CameraView3D::mouseMotionEvent(const Eigen::Vector2i &p, const Eigen::Vector2i &rel, int button, int modifiers) {
	//if (button != 1) {
	//	return true;
	//}

	if (button == 1 && tools_->isActive(Tools::MOVEMENT)) {
		rx_ += rel[0];
		ry_ += rel[1];
		pose_up_to_date_.clear();
		return true;
	} else if (tools_->isActive(Tools::ROTATE_CURSOR)) {
		auto mouse = screen()->mousePos();
		auto pos = imview_->imageCoordinateAt((mouse - mPos).cast<float>());

		Eigen::Vector3f world = ctrl_->worldAt(pos.x(), pos.y());
		ctrl_->setCursorTarget(world);
		return true;
	} else if (tools_->isActive(Tools::ROTATE_X)) {
		auto screen_origin = ctrl_->getActivePoseScreenCoord();
		double angle = atan2(float(screen_origin[1] - p[1]), float(screen_origin[0] - p[0]));
		Eigen::Affine3d rx = Eigen::Affine3d(Eigen::AngleAxisd(angle, Eigen::Vector3d(1, 0, 0)));
		ctrl_->setActivePose(rx.matrix() * cache_pose_);
	} else if (tools_->isActive(Tools::ROTATE_Y)) {
		auto screen_origin = ctrl_->getActivePoseScreenCoord();
		double angle = -atan2(float(screen_origin[1] - p[1]), float(screen_origin[0] - p[0]));
		Eigen::Affine3d ry = Eigen::Affine3d(Eigen::AngleAxisd(angle, Eigen::Vector3d(0, 1, 0)));
		ctrl_->setActivePose(ry.matrix() * cache_pose_);
	} else if (tools_->isActive(Tools::ROTATE_Z)) {
		auto screen_origin = ctrl_->getActivePoseScreenCoord();
		double angle = atan2(float(screen_origin[1] - p[1]), float(screen_origin[0] - p[0]));
		Eigen::Affine3d rz = Eigen::Affine3d(Eigen::AngleAxisd(angle, Eigen::Vector3d(0, 0, 1)));
		ctrl_->setActivePose(rz.matrix() * cache_pose_);
	} else if (tools_->isActive(Tools::TRANSLATE_X)) {
		auto mouse = screen()->mousePos();
		auto pos = imview_->imageCoordinateAt((mouse - mPos).cast<float>());
		double dx = pos[0] - double(cache_screen_[0]);
		//double dy = pos[1] - double(cache_screen_[1]);
		double dist = dx; //(std::abs(dx) > std::abs(dy)) ? dx : dy;
		Eigen::Affine3d rx = Eigen::Affine3d(Eigen::Translation3d(dist*0.001, 0.0, 0.0));
		ctrl_->setActivePose(rx.matrix() * cache_pose_);
	} else if (tools_->isActive(Tools::TRANSLATE_Y)) {
		auto mouse = screen()->mousePos();
		auto pos = imview_->imageCoordinateAt((mouse - mPos).cast<float>());
		double dx = pos[0] - double(cache_screen_[0]);
		//double dy = pos[1] - double(cache_screen_[1]);
		double dist = dx; //(std::abs(dx) > std::abs(dy)) ? dx : dy;
		Eigen::Affine3d rx = Eigen::Affine3d(Eigen::Translation3d(0.0, dist*0.001, 0.0));
		ctrl_->setActivePose(rx.matrix() * cache_pose_);
	} else if (tools_->isActive(Tools::TRANSLATE_Z)) {
		auto mouse = screen()->mousePos();
		auto pos = imview_->imageCoordinateAt((mouse - mPos).cast<float>());
		double dx = pos[0] - double(cache_screen_[0]);
		//double dy = pos[1] - double(cache_screen_[1]);
		double dist = dx; //(std::abs(dx) > std::abs(dy)) ? dx : dy;
		Eigen::Affine3d rx = Eigen::Affine3d(Eigen::Translation3d(0.0, 0.0, dist*0.001));
		ctrl_->setActivePose(rx.matrix() * cache_pose_);
	}

	//LOG(INFO) << "New pose: \n" << getUpdatedPose();
	//ctrl_->sendPose(getUpdatedPose());
	return false;
}

bool CameraView3D::scrollEvent(const Eigen::Vector2i &p, const Eigen::Vector2f &rel) {
	return true;
}

bool CameraView3D::keyboardCharacterEvent(unsigned int codepoint) {
	LOG(INFO) << "keyboardCharacterEvent: " << codepoint;
	return false;
}

Eigen::Matrix4d CameraView3D::getUpdatedPose() {
	float mspeed = ctrl_->value("mouse_speed", 0.2f);
	float rrx = ((float)ry_ * mspeed * delta_);
	float rry = (float)rx_ * mspeed * delta_;
	float rrz = 0.0;

	Eigen::Affine3d r = create_rotation_matrix(rrx, -rry, rrz);
	rotmat_ = rotmat_ * r.matrix();

	rx_ = 0;
	ry_ = 0;

	eye_[0] += (neye_[0] - eye_[0]) * lerp_speed_ * delta_;
	eye_[1] += (neye_[1] - eye_[1]) * lerp_speed_ * delta_;
	eye_[2] += (neye_[2] - eye_[2]) * lerp_speed_ * delta_;

	Eigen::Translation3d trans(eye_);
	Eigen::Affine3d t(trans);
	return t.matrix() * rotmat_;
}

void CameraView3D::processAnimation() {
	Eigen::Vector3d diff;
	diff[0] = neye_[0] - eye_[0];
	diff[1] = neye_[1] - eye_[1];
	diff[2] = neye_[2] - eye_[2];

	// Only update pose if there is enough motion
	if (diff.norm() > 0.01) {
		pose_up_to_date_.clear();
	}
}

void CameraView3D::draw(NVGcontext* ctx) {
	double now = glfwGetTime();
	delta_ = now - ftime_;
	ftime_ = now;

	processAnimation();

	// poll from ctrl_ or send on event instead?
	if (!pose_up_to_date_.test_and_set()) {
		ctrl_->sendPose(getUpdatedPose());
	}
	CameraView::draw(ctx);
}
