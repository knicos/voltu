#ifndef _FTL_GUI_CAMERA_HPP_
#define _FTL_GUI_CAMERA_HPP_

#include <ftl/rgbd/source.hpp>
#include "gltexture.hpp"

#include <string>

#ifdef HAVE_OPENVR
#include <openvr/openvr.h>
#endif

class StatisticsImage;

namespace ftl {
namespace gui {

class Screen;
class PoseWindow;

class Camera {
	public:
	Camera(ftl::gui::Screen *screen, ftl::rgbd::Source *src);
	~Camera();

	ftl::rgbd::Source *source();

	int width() { return (src_) ? src_->parameters().width : 0; }
	int height() { return (src_) ? src_->parameters().height : 0; }

	void setPose(const Eigen::Matrix4d &p);

	void mouseMovement(int rx, int ry, int button);
	void keyMovement(int key, int modifiers);

	void showPoseWindow();
	void showSettings();

	void setChannel(ftl::rgbd::channel_t c);

	void togglePause();
	void isPaused();
	const std::vector<ftl::rgbd::channel_t> &availableChannels();

	const GLTexture &captureFrame();
	const GLTexture &getLeft() const { return texture_; }

	nlohmann::json getMetaData();

	StatisticsImage *stats_ = nullptr;

	private:
	Screen *screen_;
	ftl::rgbd::Source *src_;
	GLTexture thumb_;
	GLTexture texture_;
	ftl::gui::PoseWindow *posewin_;
	nlohmann::json meta_;
	Eigen::Vector4d neye_;
	Eigen::Vector3d eye_;
	//Eigen::Vector3f orientation_;
	Eigen::Matrix4d rotmat_;
	float ftime_;
	float delta_;
	float lerpSpeed_;
	bool depth_;
	bool pause_;
	ftl::rgbd::channel_t channel_;
	std::vector<ftl::rgbd::channel_t> channels_;

	#ifdef HAVE_OPENVR
	vr::TrackedDevicePose_t rTrackedDevicePose_[ vr::k_unMaxTrackedDeviceCount ];
	#endif
};

}
}

#endif  // _FTL_GUI_CAMERA_HPP_
