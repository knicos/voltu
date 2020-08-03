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
	 * an encoded form, in which case that is used instead. If `noencode` is
	 * set to true then encoding is not performed if required and instead the
	 * channel is sent with empty data to mark availability.
	 */
	void post(ftl::data::FrameSet &fs, ftl::codecs::Channel c, bool noencode=false);

	/**
	 * Mark channel as posted without sending anything.
	 */
	void fakePost(ftl::data::FrameSet &fs, ftl::codecs::Channel c);

	/**
	 * Make the channel available in the stream even if not available locally.
	 */
	void forceAvailable(ftl::data::FrameSet &fs, ftl::codecs::Channel c);

	void post(ftl::data::Frame &f, ftl::codecs::Channel c);

	/**
	 * Encode and transmit a set of audio channels.
	 */
	//void post(const ftl::audio::FrameSet &fs);

	//void onStateChange(const std::function<void(ftl::codecs::Channel, int, int)>&);

	void onRequest(const ftl::stream::StreamCallback &);

	inline void resetSender() { do_inject_.clear(); }

	/**
	 * Force only these channels to be encoded. Any channels that already have
	 * encoders but are not in this set then have their encoders deallocated.
	 */
	void setActiveEncoders(uint32_t fsid, const std::unordered_set<ftl::codecs::Channel> &);

	void resetEncoders(uint32_t fsid);

	private:
	ftl::stream::Stream *stream_;
	int64_t timestamp_;
	int64_t injection_timestamp_=0;
	SHARED_MUTEX mutex_;
	std::atomic_flag do_inject_;
	//std::function<void(ftl::codecs::Channel, int, int)> state_cb_;
	ftl::stream::StreamCallback reqcb_;
	int add_iframes_;
	unsigned int iframe_;
	ftl::Handle handle_;

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
	std::map<uint8_t, std::pair<int64_t,unsigned int>> bitrate_map_;
	SHARED_MUTEX bitrate_mtx_;
	int bitrate_timeout_;

	//ftl::codecs::Encoder *_getEncoder(int fsid, int fid, ftl::codecs::Channel c);
	void _encodeChannel(ftl::rgbd::FrameSet &fs, ftl::codecs::Channel c, bool reset, bool last_flush);
	void _encodeChannel(ftl::data::Frame &f, ftl::codecs::Channel c, bool reset);
	void _encodeVideoChannel(ftl::rgbd::FrameSet &fs, ftl::codecs::Channel c, bool reset, bool last_flush);
	void _encodeAudioChannel(ftl::rgbd::FrameSet &fs, ftl::codecs::Channel c, bool reset, bool last_flush);
	void _encodeDataChannel(ftl::rgbd::FrameSet &fs, ftl::codecs::Channel c, bool reset, bool last_flush);
	void _encodeDataChannel(ftl::data::Frame &fs, ftl::codecs::Channel c, bool reset);

	int _generateTiles(const ftl::rgbd::FrameSet &fs, int offset, ftl::codecs::Channel c, cv::cuda::Stream &stream, bool);
	EncodingState &_getTile(int fsid, ftl::codecs::Channel c);
	cv::Rect _generateROI(const ftl::rgbd::FrameSet &fs, ftl::codecs::Channel c, int offset, bool stereo);
	float _selectFloatMax(ftl::codecs::Channel c);
	ftl::audio::Encoder *_getAudioEncoder(int fsid, int sid, ftl::codecs::Channel c, ftl::codecs::Packet &pkt);

	void _sendPersistent(ftl::data::Frame &frame);

	bool _checkNeedsIFrame(int64_t ts, bool injecting);
	uint8_t _getMinBitrate();
};

}
}

#endif  // _FTL_STREAM_SENDER_HPP_
