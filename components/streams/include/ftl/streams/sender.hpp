#ifndef _FTL_STREAM_SENDER_HPP_
#define _FTL_STREAM_SENDER_HPP_

#include <functional>
#include <ftl/rgbd/frameset.hpp>
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

	//void onStateChange(const std::function<void(ftl::codecs::Channel, int, int)>&);

	private:
	ftl::stream::Stream *stream_;
	int64_t timestamp_;
	SHARED_MUTEX mutex_;
	std::atomic_flag do_inject_;
	//std::function<void(ftl::codecs::Channel, int, int)> state_cb_;

	std::unordered_map<int, ftl::codecs::Encoder*> encoders_;

	ftl::codecs::Encoder *_getEncoder(int fsid, int fid, ftl::codecs::Channel c);
};

}
}

#endif  // _FTL_STREAM_SENDER_HPP_
