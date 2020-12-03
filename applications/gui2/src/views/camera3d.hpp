/**
 * @file camera3d.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 * @author Sebastian Hahta
 */

#pragma once

#include "../widgets/window.hpp"

#include "../view.hpp"

#include "camera.hpp"

namespace ftl {
namespace gui2 {

class CameraView3D : public CameraView {
public:
	CameraView3D(Screen *parent, Camera* ctrl);

	virtual bool keyboardEvent(int key, int scancode, int action, int modifiers) override;
	virtual bool keyboardCharacterEvent(unsigned int codepoint) override;
	virtual bool mouseMotionEvent(const Eigen::Vector2i &p, const Eigen::Vector2i &rel, int button, int modifiers) override;
	virtual bool mouseButtonEvent(const Eigen::Vector2i &p, int button, bool down, int modifiers) override;
	virtual bool scrollEvent(const Eigen::Vector2i &p, const Eigen::Vector2f &rel) override;
	virtual void draw(NVGcontext* ctx) override;

	Eigen::Matrix4d getUpdatedPose();

protected:
	// updates from keyboard
	Eigen::Vector4d neye_;
	// current
	Eigen::Vector3d eye_;
	Eigen::Matrix4d rotmat_;
	Eigen::Matrix4d cache_pose_;
	Eigen::Vector2i cache_screen_;

	// updates from mouse
	double rx_;
	double ry_;

	// times for pose update
	double ftime_;
	double delta_;

	double lerp_speed_;

	std::atomic_flag pose_up_to_date_;

	void processAnimation();

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
}
