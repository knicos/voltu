#ifndef _FTL_GUI_CAMERA_HPP_
#define _FTL_GUI_CAMERA_HPP_

#include <ftl/rgbd/frameset.hpp>
#include <ftl/render/CUDARender.hpp>
#include <ftl/render/overlay.hpp>
#include <ftl/codecs/writer.hpp>
#include "gltexture.hpp"

#include <ftl/streams/filestream.hpp>
#include <ftl/streams/sender.hpp>
#include <ftl/codecs/faces.hpp>

#include <string>
#include <array>

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
	Camera(ftl::gui::Screen *screen, int fsmask, int fid, ftl::codecs::Channel chan=ftl::codecs::Channel::Colour);
	~Camera();

	Camera(const Camera &)=delete;

	int width() const { return width_; }
	int height() const { return height_; }

	int getFramesetMask() const { return fsmask_; }

	bool usesFrameset(int id) const { return fsmask_ & (1 << id); }

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
	inline bool isStereo() const { return stereo_; }

	void setStereo(bool v);

	/**
	 * Main function to obtain latest frames.
	 */
	void update(std::vector<ftl::rgbd::FrameSet *> &fss);

	/**
	 * Update the available channels.
	 */
	void update(int fsid, const ftl::codecs::Channels<0> &c);

	/**
	 * Draw virtual camera only if the frameset has been updated since last
	 * draw.
	 */
	void drawUpdated(std::vector<ftl::rgbd::FrameSet*> &fss);

	void draw(std::vector<ftl::rgbd::FrameSet*> &fss);

	void drawOverlay(const Eigen::Vector2f &);

	inline int64_t getFrameTimeMS() const { return int64_t(delta_ * 1000.0f); }

	const ftl::rgbd::Camera &getIntrinsics() const { return state_.getLeft(); }
	const Eigen::Matrix4d &getPose() const { return state_.getPose(); }

	/**
	 * @internal. Used to inform the camera if it is the active camera or not.
	 */
	void active(bool);

	const void captureFrame();
	const GLTexture &getLeft() const { return texture1_; }
	const GLTexture &getRight() const { return texture2_; }
	const GLTexture &getDepth() const { return depth1_; }

	void snapshot(const std::string &filename);

	void startVideoRecording(const std::string &filename);

	void stopVideoRecording();

	//nlohmann::json getMetaData();

	const std::string &name() const { return name_; }

	StatisticsImage *stats_ = nullptr;

	float getDepth(int x, int y);
	float getDepth(float x, float y) { return getDepth((int) round(x), (int) round(y)); }
	cv::Point3f getPoint(int x, int y);
	cv::Point3f getPoint(float x, float y) { return getPoint((int) round(x), (int) round(y)); }
	//cv::Point3f getNormal(int x, int y);
	//cv::Point3f getNormal(float x, float y) { return getNormal((int) round(x), (int) round(y)); }
	void setTransform(const Eigen::Matrix4d &T);
	Eigen::Matrix4d getTransform() const;
	const std::vector<std::string> &getMessages() const { return msgs_; }

#ifdef HAVE_OPENVR
	bool isVR() { return vr_mode_; }
	bool setVR(bool on);
#else
	bool isVR() { return false; }
#endif

	private:

	Screen *screen_;
	unsigned int fsmask_;  // Frameset Mask
	int fid_;

	int width_;
	int height_;

	GLTexture texture1_; // first channel (always left at the moment)
	GLTexture texture2_; // second channel ("right")
	GLTexture depth1_;

	ftl::gui::PoseWindow *posewin_;
	//nlohmann::json meta_;
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
	
	cv::cuda::HostMem im_depth_;
	//cv::cuda::HostMem im_normals_;
	cv::Mat im_normals_f_;

	cv::Mat overlay_; // first channel (left)
	bool stereo_;
	std::atomic_flag stale_frame_;
	int rx_;
	int ry_;
	std::vector<ftl::rgbd::FrameSet*> *framesets_;

	ftl::render::CUDARender *renderer_;
	ftl::render::CUDARender *renderer2_;
	ftl::render::Colouriser *colouriser_;
	ftl::overlay::Overlay *overlayer_;

	ftl::Configurable *intrinsics_;
	ftl::operators::Graph *post_pipe_;
	ftl::rgbd::Frame frame_;
	ftl::rgbd::FrameState state_;
	ftl::stream::File *record_stream_;
	ftl::stream::Sender *record_sender_;

	std::string name_;

	std::vector<std::string> msgs_;

	int transform_ix_;
	std::array<Eigen::Matrix4d,ftl::stream::kMaxStreams> transforms_;  // Frameset transforms for virtual cam
	Eigen::Matrix4d T_ = Eigen::Matrix4d::Identity();

	MUTEX mutex_;

	#ifdef HAVE_OPENVR
	vr::TrackedDevicePose_t rTrackedDevicePose_[ vr::k_unMaxTrackedDeviceCount ];
	bool vr_mode_;
	float baseline_;
	#endif

	void _downloadFrames(ftl::cuda::TextureObject<uchar4> &, ftl::cuda::TextureObject<uchar4> &);
	void _downloadFrames(ftl::cuda::TextureObject<uchar4> &);
	void _downloadFrames();
	void _draw(std::vector<ftl::rgbd::FrameSet*> &fss);
	void _applyPoseEffects(std::vector<ftl::rgbd::FrameSet*> &fss);
	std::pair<const ftl::rgbd::Frame *, const ftl::codecs::Face *> _selectFace(std::vector<ftl::rgbd::FrameSet*> &fss);
	void _generateWindow(const ftl::rgbd::Frame &, const ftl::codecs::Face &face, Eigen::Matrix4d &pose_adjust, ftl::render::ViewPort &vp);
};

}
}

#endif  // _FTL_GUI_CAMERA_HPP_
