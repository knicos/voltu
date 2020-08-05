#ifndef _FTL_DATA_TRANSCODER_HPP_
#define _FTL_DATA_TRANSCODER_HPP_

#include <functional>
#include <ftl/rgbd/frameset.hpp>
#include <ftl/audio/frameset.hpp>
#include <ftl/streams/stream.hpp>
#include <ftl/codecs/decoder.hpp>
#include <ftl/audio/decoder.hpp>
#include <ftl/streams/builder.hpp>

namespace ftl {
namespace stream {

/**
 * Convert packet streams into framesets.
 */
class Receiver : public ftl::Configurable, public ftl::data::Generator {
	public:
	explicit Receiver(nlohmann::json &config, ftl::data::Pool *);
	~Receiver();

	void setStream(ftl::stream::Stream*);

	/**
	 * Loop a response frame back into a local buffer. Should only be called
	 * for local builder framesets and probably only by `Feed`. It takes all
	 * changes in the frame and puts them as `createChange` in the next
	 * buffered frame in builder. The response frame is empty afterwards as
	 * the data is moved, not copied.
	 */
	void loopback(ftl::data::Frame &, ftl::codecs::Channel);

	/**
	 * Register a callback for received framesets. Sources are automatically
	 * created to match the data received.
	 */
	ftl::Handle onFrameSet(const ftl::data::FrameSetCallback &cb) override;

	ftl::streams::BaseBuilder &builder(uint32_t id);

	void registerBuilder(const std::shared_ptr<ftl::streams::BaseBuilder> &b);

	void processPackets(const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt);

	void removeBuilder(uint32_t id);

	private:
	ftl::stream::Stream *stream_;
	ftl::data::Pool *pool_;
	ftl::SingletonHandler<const ftl::data::FrameSetPtr&> callback_;
	std::unordered_map<uint32_t, std::shared_ptr<ftl::streams::BaseBuilder>> builders_;
	std::unordered_map<uint32_t, ftl::Handle> handles_;
	ftl::codecs::Channel second_channel_;
	int64_t timestamp_;
	SHARED_MUTEX mutex_;
	unsigned int frame_mask_;
	ftl::Handle handle_;

	struct InternalVideoStates {
		InternalVideoStates();

		int64_t timestamp;
		//ftl::rgbd::Frame frame;
		ftl::codecs::Decoder* decoders[32];
		cv::cuda::GpuMat surface[32];
		MUTEX mutex;
		ftl::codecs::Channels<0> completed;
		int width=0;
		int height=0;
	};

	struct InternalAudioStates {
		InternalAudioStates();

		int64_t timestamp;
		//ftl::audio::Frame frame;
		MUTEX mutex;
		ftl::codecs::Channels<0> completed;
		ftl::audio::Decoder *decoder;
	};

	std::vector<InternalVideoStates*> video_frames_[ftl::stream::kMaxStreams];
	std::vector<InternalAudioStates*> audio_frames_[ftl::stream::kMaxStreams];

	void _processConfig(InternalVideoStates &frame, const ftl::codecs::Packet &pkt);
	void _processState(const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt);
	void _processData(const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt);
	void _processAudio(const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt);
	void _processVideo(const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt);
	void _createDecoder(InternalVideoStates &frame, int chan, const ftl::codecs::Packet &pkt);
	ftl::audio::Decoder *_createAudioDecoder(InternalAudioStates &frame, const ftl::codecs::Packet &pkt);
	InternalVideoStates &_getVideoFrame(const ftl::codecs::StreamPacket &spkt, int ix=0);
	InternalAudioStates &_getAudioFrame(const ftl::codecs::StreamPacket &spkt, int ix=0);
	void _finishPacket(ftl::streams::LockedFrameSet &fs, size_t fix);
};

}
}

#endif  // _FTL_DATA_TRANSCODER_HPP_
