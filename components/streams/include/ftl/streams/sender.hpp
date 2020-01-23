#ifndef _FTL_STREAM_SENDER_HPP_
#define _FTL_STREAM_SENDER_HPP_

#include <functional>
#include <ftl/rgbd/frameset.hpp>
#include <ftl/audio/frameset.hpp>
#include <ftl/streams/stream.hpp>
#include <ftl/codecs/encoder.hpp>

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
	void post(const ftl::rgbd::FrameSet &fs);

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

	struct EncodingState {
		uint8_t bitrate;
		ftl::codecs::Encoder *encoder[2];
		cv::cuda::GpuMat surface;
		cudaStream_t stream;
	};

	std::unordered_map<int, EncodingState> state_;

	//ftl::codecs::Encoder *_getEncoder(int fsid, int fid, ftl::codecs::Channel c);
	void _encodeChannel(const ftl::rgbd::FrameSet &fs, ftl::codecs::Channel c, bool reset);
	int _generateTiles(const ftl::rgbd::FrameSet &fs, int offset, ftl::codecs::Channel c, cv::cuda::Stream &stream, bool);
	EncodingState &_getTile(int fsid, ftl::codecs::Channel c);
	cv::Rect _generateROI(const ftl::rgbd::FrameSet &fs, ftl::codecs::Channel c, int offset);
	float _selectFloatMax(ftl::codecs::Channel c);
};

}
}

#endif  // _FTL_STREAM_SENDER_HPP_
