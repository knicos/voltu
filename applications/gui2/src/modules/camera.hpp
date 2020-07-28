#pragma once

#include "../module.hpp"
#include "../screen.hpp"
#include "../views/camera.hpp"

#include <ftl/render/colouriser.hpp>
#include <ftl/render/overlay.hpp>
#include <ftl/codecs/touch.hpp>

namespace ftl {
namespace gui2 {

class Camera : public Module {
public:
	using Module::Module;

	virtual void init() override;
	virtual void update(double delta) override;

	virtual void activate(ftl::data::FrameID id);
	void setChannel(ftl::codecs::Channel c);
	void setPaused(bool set) { paused = set; };
	bool isPaused() { return paused; }

	float volume();
	void setVolume(float v);

	/** Gets current active frame to display. Always 4 channel uchar4. Reference
	 * will stay valid until getFrame() is called again. Always returns a
	 * reference to internal buffer. */
	ftl::cuda::TextureObject<uchar4>& getFrame();
	bool getFrame(ftl::cuda::TextureObject<uchar4>&);

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

	ftl::render::Colouriser* colouriser() { return colouriser_.get(); };
	ftl::overlay::Overlay* overlay() { return overlay_.get(); }

	void drawOverlay(NVGcontext *ctx);

	std::string getActiveSourceURI();

	float depthAt(int x, int y);

	bool isRecording();
	void stopRecording();
	void startRecording(const std::string &filename, const std::unordered_set<ftl::codecs::Channel> &channels);
	void startStreaming(const std::unordered_set<ftl::codecs::Channel> &channels);

	void snapshot(const std::string &filename);

private:
	int frame_idx = -1;
	ftl::data::FrameID frame_id_;
	ftl::codecs::Channel channel = ftl::codecs::Channel::Colour;
	ftl::stream::Feed::Filter *filter = nullptr;
	std::atomic_bool paused = false; // TODO: implement in InputOutput
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

	ftl::data::FrameSetPtr current_fs_;
	ftl::data::FrameSetPtr latest_;
	ftl::cuda::TextureObject<uchar4> current_frame_;

	std::unique_ptr<ftl::render::Colouriser> colouriser_;
	std::unique_ptr<ftl::overlay::Overlay> overlay_;

	std::map<ftl::data::Message,std::string> messages_;

	CameraView* view = nullptr;

	MUTEX mtx_;

	void initiate_(ftl::data::Frame &frame);
	void _updateCapabilities(ftl::data::Frame &frame);
};

}
}