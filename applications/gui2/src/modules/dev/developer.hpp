#pragma once

#include "../../module.hpp"
#include "../camera.hpp"
#include <ftl/threads.hpp>
#include <ftl/data/new_frame.hpp>
#include <ftl/disparity/features.hpp>

namespace ftl {
namespace gui2 {

class DisparityView;

class Developer : public Module {
	public:
	using Module::Module;
	void init() override;

	virtual ~Developer();
};

class DisparityDev : public Module {
	public:
	using Module::Module;
	void init() override;

	virtual ~DisparityDev();

	virtual void activate(ftl::data::FrameID id);

	/** Gets current active frame to display. Always 4 channel uchar4. Reference
	 * will stay valid until getFrame() is called again. Always returns a
	 * reference to internal buffer. */
	ftl::cuda::TextureObject<uchar4>& getFrame();
	ftl::cuda::TextureObject<uchar4>& getFrame(ftl::codecs::Channel channel);
	bool getFrame(ftl::cuda::TextureObject<uchar4>&);
	bool getFrame(ftl::cuda::TextureObject<uchar4>&, ftl::codecs::Channel channel);

	/** Check if new frame is available */
	bool hasFrame();

	inline bool isLive() const { return live_; }
	inline bool isTouchable() const { return touch_; }
	inline bool isMovable() const { return movable_; }
	inline bool isVR() const { return vr_; }

	void setFocalPoint(int x, int y);
	//void setMode(ftl::disparity::Mode m);
	const cv::cuda::GpuMat& getFeatureImageLeft(ftl::disparity::ColourFeatures::Feature f);
	const cv::cuda::GpuMat& getFeatureImageRight(ftl::disparity::ColourFeatures::Feature f);
	//ftl::cuda::TextureObject<uchar4>& getHistogramImage(ftl::disparity::Histogram h);
	//ftl::cuda::TextureObject<uchar4>& getDisparityImage();
	//ftl::cuda::TextureObject<uchar4>& getErrorImage();
	//ftl::cuda::TextureObject<uchar4>& getConfidenceImage();

	void generate();

	double getLastRuntime();

	private:
	int frame_idx = -1;
	ftl::data::FrameID frame_id_;
	ftl::stream::Feed::Filter *filter_ = nullptr;
	std::atomic_bool paused_ = false; // TODO: implement in InputOutput
	bool has_seen_frame_ = false;
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
	ftl::cuda::TextureObject<uchar4> current_frame_colour_;

	std::unique_ptr<ftl::render::Colouriser> colouriser_;
	std::unique_ptr<ftl::overlay::Overlay> overlay_;

	cv::cuda::GpuMat left_;
	cv::cuda::GpuMat right_;

	ftl::disparity::ColourFeatures col_feat_left_;
	ftl::disparity::ColourFeatures col_feat_right_;



	std::map<ftl::data::Message,std::string> messages_;

	DisparityView* view = nullptr;

	MUTEX mtx_;

	void initiate_(ftl::data::Frame &frame);
	void _updateCapabilities(ftl::data::Frame &frame);
};

}
}