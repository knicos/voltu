#ifndef _FTL_GUI_CAMERA_HPP_
#define _FTL_GUI_CAMERA_HPP_

#include <ftl/rgbd/source.hpp>
#include "gltexture.hpp"

#include <string>

class StatisticsImage;

namespace ftl {
namespace gui {

class Screen;
class PoseWindow;

class Camera {
	public:
	Camera(ftl::gui::Screen *screen, ftl::rgbd::Source *src);
	~Camera();

	Camera(const Camera &)=delete;

	ftl::rgbd::Source *source();

	int width() { return (src_) ? src_->parameters().width : 0; }
	int height() { return (src_) ? src_->parameters().height : 0; }

	void setPose(const Eigen::Matrix4d &p);

	void mouseMovement(int rx, int ry, int button);
	void keyMovement(int key, int modifiers);

	void showPoseWindow();
	void showSettings();

	void setChannel(ftl::rgbd::Channel c);

	void togglePause();
	void isPaused();
	const ftl::rgbd::Channels &availableChannels();

	const GLTexture &captureFrame();

	bool thumbnail(cv::Mat &thumb);

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
	bool sdepth_;
	bool pause_;
	ftl::rgbd::Channel channel_;
	ftl::rgbd::Channels channels_;
	cv::Mat rgb_;
	cv::Mat depth_;
	MUTEX mutex_;
};

}
}

#endif  // _FTL_GUI_CAMERA_HPP_
