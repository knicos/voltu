#pragma once

#include "../module.hpp"
#include "../screen.hpp"
#include "../views/camera.hpp"

#include <ftl/render/colouriser.hpp>
#include <ftl/render/overlay.hpp>
#include <ftl/codecs/touch.hpp>
#include <ftl/audio/mixer.hpp>

namespace ftl {
namespace gui2 {

class Camera : public Module {
public:
	using Module::Module;

	virtual void init() override;
	virtual void update(double delta) override;

	virtual void activate(ftl::data::FrameID id);
	void setChannel(ftl::codecs::Channel c);
	void setPaused(bool set);
	bool isPaused() const { return paused_; }

	void toggleOverlay();

	float volume();
	void setVolume(float v);

	/** Gets current active frame to display. Always 4 channel uchar4. Reference
	 * will stay valid until getFrame() is called again. Always returns a
	 * reference to internal buffer. */
	ftl::cuda::TextureObject<uchar4>& getFrame();
	ftl::cuda::TextureObject<uchar4>& getFrame(ftl::codecs::Channel channel);
	bool getFrame(ftl::cuda::TextureObject<uchar4>&);
	bool getFrame(ftl::cuda::TextureObject<uchar4>&, ftl::codecs::Channel channel);

	std::unordered_set<ftl::codecs::Channel> availableChannels();

	/** This includes data channels etc */
	std::unordered_set<ftl::codecs::Channel> allAvailableChannels();

	void touch(int id, ftl::codecs::TouchType t, int x, int y, float d, int strength);

	/** Check if new frame is available */
	bool hasFrame();
	void sendPose(const Eigen::Matrix4d &pose);

	inline bool isLive() const { return live_; }
	inline bool isTouchable() const { return touch_; }
	inline bool isMovable() const { return movable_; }
	inline bool isVR() const { return vr_; }

	ftl::render::Colouriser* colouriser() { return colouriser_.get(); };
	ftl::overlay::Overlay* overlay() { return overlay_.get(); }
	ftl::audio::StereoMixerF<100> *mixer();

	void drawOverlay(NVGcontext *ctx, const nanogui::Vector2f &size, const nanogui::Vector2f &is, const Eigen::Vector2f &offset);

	std::string getActiveSourceURI();

	float depthAt(int x, int y);
	Eigen::Vector3f worldAt(int x, int y);

	bool isRecording();
	void stopRecording();
	void startRecording(const std::string &filename, const std::unordered_set<ftl::codecs::Channel> &channels);
	void startStreaming(const std::unordered_set<ftl::codecs::Channel> &channels);

	void snapshot(const std::string &filename);

	const Eigen::Matrix4d &cursor() const;

	void setCursorPosition(const Eigen::Vector3f &pos) { cursor_pos_ = pos; cursor_ = _cursor(); }
	void setCursorNormal(const Eigen::Vector3f &norm) { cursor_normal_ = norm; cursor_ = _cursor(); }
	void setCursorTarget(const Eigen::Vector3f &targ);
	void setCursor(int x, int y);

	const Eigen::Vector3f getCursorPosition() const { return cursor_pos_; }

	void setOriginToCursor();
	void resetOrigin();
	void saveCursorToPoser();

	Eigen::Matrix4d getActivePose();
	nanogui::Vector2i getActivePoseScreenCoord();
	void transformActivePose(const Eigen::Matrix4d &pose);
	void setActivePose(const Eigen::Matrix4d &pose);

private:
	int frame_idx = -1;
	ftl::data::FrameID frame_id_;
	ftl::codecs::Channel channel_ = ftl::codecs::Channel::Colour;
	ftl::stream::Feed::Filter *filter_ = nullptr;
	std::atomic_bool paused_ = false; // TODO: implement in InputOutput
	bool has_seen_frame_ = false;
	ftl::codecs::Touch point_;
	bool live_=false;
	bool touch_=false;
	bool movable_=false;
	bool vr_=false;
	float last_=0.0f;
	std::atomic_int16_t nframes_=0;
	std::atomic_int64_t latency_=0;
	int update_fps_freq_=30; // fps counter update frequency (frames)
	Eigen::Vector3f cursor_pos_;
	Eigen::Vector3f cursor_target_;
	Eigen::Vector3f cursor_normal_;
	int cursor_save_id_=0;
	Eigen::Matrix4d cursor_;

	ftl::data::FrameSetPtr current_fs_;
	ftl::data::FrameSetPtr latest_;
	ftl::cuda::TextureObject<uchar4> current_frame_;
	ftl::cuda::TextureObject<uchar4> current_frame_colour_;

	std::unique_ptr<ftl::render::Colouriser> colouriser_;
	std::unique_ptr<ftl::overlay::Overlay> overlay_;

	std::map<ftl::data::Message,std::string> messages_;

	CameraView* view = nullptr;
	ftl::audio::StereoMixerF<100> *mixer_ = nullptr;

	MUTEX mtx_;

	void initiate_(ftl::data::Frame &frame);
	void _updateCapabilities(ftl::data::Frame &frame);
	Eigen::Matrix4d _cursor() const;
};

}
}
