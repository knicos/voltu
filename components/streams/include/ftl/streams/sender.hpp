#ifndef _FTL_STREAM_SENDER_HPP_
#define _FTL_STREAM_SENDER_HPP_

#include <functional>
#include <ftl/rgbd/frameset.hpp>
#include <ftl/audio/frameset.hpp>
#include <ftl/streams/stream.hpp>
#include <ftl/codecs/encoder.hpp>
#include <ftl/audio/encoder.hpp>

#include <unordered_map>

namespace ftl {
namespace stream {

/**
 * Convert framesets into packet streams.
 */
class Sender : public ftl::Configurable {
	public:
	explicit Sender(nlohmann::json &config);
	~Sender();

	void setStream(ftl::stream::Stream*);

	/**
	 * Encode and transmit an entire frame set. Frames may already contain
	 * an encoded form, in which case that is used instead.
	 */
	void post(ftl::rgbd::FrameSet &fs);

	/**
	 * Encode and transmit a set of audio channels.
	 */
	void post(const ftl::audio::FrameSet &fs);

	//void onStateChange(const std::function<void(ftl::codecs::Channel, int, int)>&);

	void onRequest(const ftl::stream::StreamCallback &);

	private:
	ftl::stream::Stream *stream_;
	int64_t timestamp_;
	SHARED_MUTEX mutex_;
	std::atomic_flag do_inject_;
	//std::function<void(ftl::codecs::Channel, int, int)> state_cb_;
	ftl::stream::StreamCallback reqcb_;
	int add_iframes_;
	int iframe_;

	struct EncodingState {
		uint8_t bitrate;
		ftl::codecs::Encoder *encoder[2];
		cv::cuda::GpuMat surface;
		cudaStream_t stream;
	};

	struct AudioState {
		ftl::audio::Encoder *encoder;
	};

	std::unordered_map<int, EncodingState> state_;
	std::unordered_map<int, AudioState> audio_state_;

	//ftl::codecs::Encoder *_getEncoder(int fsid, int fid, ftl::codecs::Channel c);
	void _encodeChannel(ftl::rgbd::FrameSet &fs, ftl::codecs::Channel c, bool reset);
	int _generateTiles(const ftl::rgbd::FrameSet &fs, int offset, ftl::codecs::Channel c, cv::cuda::Stream &stream, bool, bool);
	EncodingState &_getTile(int fsid, ftl::codecs::Channel c);
	cv::Rect _generateROI(const ftl::rgbd::FrameSet &fs, ftl::codecs::Channel c, int offset, bool stereo);
	float _selectFloatMax(ftl::codecs::Channel c);
	ftl::audio::Encoder *_getAudioEncoder(int fsid, int sid, ftl::codecs::Channel c, ftl::codecs::Packet &pkt);
};

}
}

#endif  // _FTL_STREAM_SENDER_HPP_
