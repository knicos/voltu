#ifndef _FTL_GUI_CAMERA_HPP_
#define _FTL_GUI_CAMERA_HPP_

#include <ftl/rgbd/source.hpp>
#include <ftl/codecs/writer.hpp>
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

	Camera(const Camera &)=delete;

	ftl::rgbd::Source *source();

	int width() { return (src_) ? src_->parameters().width : 0; }
	int height() { return (src_) ? src_->parameters().height : 0; }

	void setPose(const Eigen::Matrix4d &p);

	void mouseMovement(int rx, int ry, int button);
	void keyMovement(int key, int modifiers);

	void showPoseWindow();
	void showSettings();

	void setChannel(ftl::codecs::Channel c);
	const ftl::codecs::Channel getChannel() { return channel_; }
	
	void togglePause();
	void isPaused();
	const ftl::codecs::Channels &availableChannels() { return channels_; }

	const GLTexture &captureFrame();
	const GLTexture &getLeft() const { return texture1_; }
	const GLTexture &getRight() const { return texture2_; }

	bool thumbnail(cv::Mat &thumb);

	void snapshot();

	void toggleVideoRecording();

	nlohmann::json getMetaData();

	StatisticsImage *stats_ = nullptr;


#ifdef HAVE_OPENVR
	bool isVR() { return vr_mode_; }
	bool setVR(bool on);
#else
	bool isVR() { return false; }
#endif

	private:
	Screen *screen_;
	ftl::rgbd::Source *src_;
	GLTexture thumb_;
	GLTexture texture1_; // first channel (always left at the moment)
	GLTexture texture2_; // second channel ("right")

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
	ftl::codecs::Channel channel_;
	ftl::codecs::Channels channels_;
	cv::Mat im1_; // first channel (left)
	cv::Mat im2_; // second channel ("right")
	bool recording_;
	std::ofstream *fileout_;
	ftl::codecs::Writer *writer_;
	ftl::rgbd::RawCallback recorder_;

	MUTEX mutex_;

	#ifdef HAVE_OPENVR
	vr::TrackedDevicePose_t rTrackedDevicePose_[ vr::k_unMaxTrackedDeviceCount ];
	bool vr_mode_;
	float baseline_;
	#endif
};

}
}

#endif  // _FTL_GUI_CAMERA_HPP_
