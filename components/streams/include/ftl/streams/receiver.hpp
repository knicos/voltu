#ifndef _FTL_DATA_TRANSCODER_HPP_
#define _FTL_DATA_TRANSCODER_HPP_

#include <functional>
#include <ftl/rgbd/frameset.hpp>
#include <ftl/audio/frameset.hpp>
#include <ftl/streams/stream.hpp>
#include <ftl/codecs/decoder.hpp>

namespace ftl {
namespace stream {

/**
 * Convert packet streams into framesets.
 */
class Receiver : public ftl::Configurable, public ftl::rgbd::Generator {
	public:
	explicit Receiver(nlohmann::json &config);
	~Receiver();

	void setStream(ftl::stream::Stream*);

	/**
	 * Encode and transmit an entire frame set. Frames may already contain
	 * an encoded form, in which case that is used instead.
	 */
	//void post(const ftl::rgbd::FrameSet &fs);

	// void write(const ftl::audio::FrameSet &f);

	size_t size() override;

	ftl::rgbd::FrameState &state(size_t ix) override;

	/**
	 * Register a callback for received framesets. Sources are automatically
	 * created to match the data received.
	 */
	void onFrameSet(const ftl::rgbd::VideoCallback &cb) override;

	/**
	 * Add a frameset handler to a specific stream ID.
	 */
	void onFrameSet(size_t s, const ftl::rgbd::VideoCallback &cb);

	void onAudio(const ftl::audio::FrameSet::Callback &cb);

	private:
	ftl::stream::Stream *stream_;
	ftl::rgbd::VideoCallback fs_callback_;
	ftl::audio::FrameSet::Callback audio_cb_;
	ftl::rgbd::Builder builder_[ftl::stream::kMaxStreams];
	ftl::codecs::Channel second_channel_;
	int64_t timestamp_;
	SHARED_MUTEX mutex_;
	unsigned int frame_mask_;

	struct InternalVideoStates {
		InternalVideoStates();

		int64_t timestamp;
		ftl::rgbd::FrameState state;
		ftl::rgbd::Frame frame;
		ftl::codecs::Decoder* decoders[32];
		cv::cuda::GpuMat surface[32];
		MUTEX mutex;
		ftl::codecs::Channels<0> completed;
	};

	struct InternalAudioStates {
		InternalAudioStates();

		int64_t timestamp;
		ftl::audio::FrameState state;
		ftl::audio::Frame frame;
		MUTEX mutex;
		ftl::codecs::Channels<0> completed;
	};

	std::vector<InternalVideoStates*> video_frames_[ftl::stream::kMaxStreams];
	std::vector<InternalAudioStates*> audio_frames_[ftl::stream::kMaxStreams];

	void _processConfig(InternalVideoStates &frame, const ftl::codecs::Packet &pkt);
	void _processState(const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt);
	void _processData(const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt);
	void _processAudio(const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt);
	void _processVideo(const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt);
	void _createDecoder(InternalVideoStates &frame, int chan, const ftl::codecs::Packet &pkt);
	InternalVideoStates &_getVideoFrame(const ftl::codecs::StreamPacket &spkt, int ix=0);
	InternalAudioStates &_getAudioFrame(const ftl::codecs::StreamPacket &spkt, int ix=0);
};

}
}

#endif  // _FTL_DATA_TRANSCODER_HPP_
