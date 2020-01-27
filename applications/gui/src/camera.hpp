#ifndef _FTL_GUI_CAMERA_HPP_
#define _FTL_GUI_CAMERA_HPP_

#include <ftl/rgbd/frameset.hpp>
#include <ftl/render/tri_render.hpp>
#include <ftl/codecs/writer.hpp>
#include "gltexture.hpp"

#include <ftl/streams/filestream.hpp>
#include <ftl/streams/sender.hpp>

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
	Camera(ftl::gui::Screen *screen, int fsid, int fid, ftl::codecs::Channel chan=ftl::codecs::Channel::Colour);
	~Camera();

	Camera(const Camera &)=delete;

	int width() const { return width_; }
	int height() const { return height_; }

	void setPose(const Eigen::Matrix4d &p);

	void mouseMovement(int rx, int ry, int button);
	void keyMovement(int key, int modifiers);

	void showPoseWindow();
	void showSettings();

	void setChannel(ftl::codecs::Channel c);
	const ftl::codecs::Channel getChannel() { return channel_; }
	
	void togglePause();
	void isPaused();
	inline bool isVirtual() const { return fid_ == 255; }
	const ftl::codecs::Channels<0> &availableChannels() { return channels_; }

	/**
	 * Main function to obtain latest frames.
	 */
	void update(ftl::rgbd::FrameSet &fs);

	/**
	 * Update the available channels.
	 */
	void update(const ftl::codecs::Channels<0> &c) { channels_ = (isVirtual()) ? c + ftl::codecs::Channel::Right : c; }

	void draw(ftl::rgbd::FrameSet &fs);

	const GLTexture &captureFrame();
	const GLTexture &getLeft() const { return texture1_; }
	const GLTexture &getRight() const { return texture2_; }

	bool thumbnail(cv::Mat &thumb);

	void snapshot(const std::string &filename);

	void startVideoRecording(const std::string &filename);

	void stopVideoRecording();

	nlohmann::json getMetaData();

	const std::string &name() const { return name_; }

	StatisticsImage *stats_ = nullptr;


#ifdef HAVE_OPENVR
	bool isVR() { return vr_mode_; }
	bool setVR(bool on);
#else
	bool isVR() { return false; }
#endif

	private:
	cv::Mat visualizeActiveChannel();

	Screen *screen_;
	int fsid_;
	int fid_;

	int width_;
	int height_;

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
	ftl::codecs::Channels<0> channels_;
	cv::Mat im1_; // first channel (left)
	cv::Mat im2_; // second channel ("right")

	ftl::render::Triangular *renderer_;
	ftl::rgbd::Frame frame_;
	ftl::rgbd::FrameState state_;
	ftl::stream::File *record_stream_;
	ftl::stream::Sender *record_sender_;

	std::string name_;

	MUTEX mutex_;

	#ifdef HAVE_OPENVR
	vr::TrackedDevicePose_t rTrackedDevicePose_[ vr::k_unMaxTrackedDeviceCount ];
	bool vr_mode_;
	float baseline_;
	#endif

	void _downloadFrames(ftl::rgbd::Frame *frame);
	void _draw(ftl::rgbd::FrameSet &fs);
};

}
}

#endif  // _FTL_GUI_CAMERA_HPP_
