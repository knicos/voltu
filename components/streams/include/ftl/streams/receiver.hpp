#ifndef _FTL_DATA_TRANSCODER_HPP_
#define _FTL_DATA_TRANSCODER_HPP_

#include <functional>
#include <ftl/rgbd/frameset.hpp>
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

	ftl::rgbd::FrameState &state(int ix) override;

	/**
	 * Register a callback for received framesets. Sources are automatically
	 * created to match the data received.
	 */
	void onFrameSet(const ftl::rgbd::VideoCallback &cb) override;

	// void onFrameSet(const AudioCallback &cb);

	private:
	ftl::stream::Stream *stream_;
	ftl::rgbd::VideoCallback fs_callback_;
	ftl::rgbd::Builder builder_;
	ftl::codecs::Channel second_channel_;
	int64_t timestamp_;
	SHARED_MUTEX mutex_;

	struct InternalStates {
		InternalStates();

		int64_t timestamp;
		ftl::rgbd::FrameState state;
		ftl::rgbd::Frame frame;
		ftl::codecs::Decoder* decoders[32];
		MUTEX mutex;
		ftl::codecs::Channels<0> completed;
	};

	std::vector<InternalStates*> frames_;

	void _processConfig(InternalStates &frame, const ftl::codecs::Packet &pkt);
	void _createDecoder(InternalStates &frame, int chan, const ftl::codecs::Packet &pkt);
	InternalStates &_getFrame(const ftl::codecs::StreamPacket &spkt);
};

}
}

#endif  // _FTL_DATA_TRANSCODER_HPP_
