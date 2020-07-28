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

	setZoom(false);
	setPan(false);
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
	return CameraView::mouseButtonEvent(p, button, down, modifiers);
}

bool CameraView3D::mouseMotionEvent(const Eigen::Vector2i &p, const Eigen::Vector2i &rel, int button, int modifiers) {
	if (button != 1) {
		return true;
	}

	rx_ += rel[0];
	ry_ += rel[1];
	pose_up_to_date_.clear();

	//LOG(INFO) << "New pose: \n" << getUpdatedPose();
	//ctrl_->sendPose(getUpdatedPose());
	return true;
}

bool CameraView3D::scrollEvent(const Eigen::Vector2i &p, const Eigen::Vector2f &rel) {
	return true;
}

bool CameraView3D::keyboardCharacterEvent(unsigned int codepoint) {
	LOG(INFO) << "keyboardCharacterEvent: " << codepoint;
	return false;
}

Eigen::Matrix4d CameraView3D::getUpdatedPose() {
	float rrx = ((float)ry_ * 0.2f * delta_);
	float rry = (float)rx_ * 0.2f * delta_;
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
