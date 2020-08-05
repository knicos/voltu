#ifndef _FTL_STREAM_NETSTREAM_HPP_
#define _FTL_STREAM_NETSTREAM_HPP_

#include <ftl/net/universe.hpp>
#include <ftl/threads.hpp>
#include <ftl/codecs/packet.hpp>
#include <ftl/streams/stream.hpp>
#include <ftl/handle.hpp>
#include <string>

namespace ftl {
namespace stream {

class AdaptiveBitrate;

namespace detail {
struct StreamClient {
	ftl::UUID peerid;
	std::atomic<int> txcount;	// Frames sent since last request
	int txmax;					// Frames to send in request
	uint8_t quality;
};
}

/**
 * The maximum number of frames a client can request in a single request.
 */
static const int kMaxFrames = 100;

/**
 * Send and receive packets over a network. This class manages the connection
 * of clients or the discovery of a stream and deals with bitrate adaptations.
 * Each packet post is forwarded to each connected client that is still active.
 */
class Net : public Stream {
	public:
	Net(nlohmann::json &config, ftl::net::Universe *net);
	~Net();

	bool post(const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &) override;

	bool begin() override;
	bool end() override;
	bool active() override;

	void reset() override;

	inline const ftl::UUID &getPeer() const { return peer_; }

	inline ftl::Handle onClientConnect(const std::function<bool(ftl::net::Peer*)> &cb) { return connect_cb_.on(cb); }

	/**
	 * Return the average bitrate of all streams since the last call to this
	 * function. Units are Mbps.
	 */
	static float getRequiredBitrate();

	private:
	SHARED_MUTEX mutex_;
	bool active_;
	ftl::net::Universe *net_;
	int64_t clock_adjust_;
	ftl::UUID time_peer_;
	ftl::UUID peer_;
	int64_t last_frame_;
	int64_t last_ping_;
	std::string uri_;
	std::string base_uri_;
	bool host_;
	int tally_;
	std::array<std::atomic<int>,32> reqtally_;
	std::unordered_set<ftl::codecs::Channel> last_selected_;
	uint8_t bitrate_=255;
	std::atomic_int64_t bytes_received_ = 0;
	int64_t last_completion_ = 0;
	int64_t time_at_last_ = 0;
	float required_bps_;
	float actual_bps_;
	bool abr_enabled_;

	AdaptiveBitrate *abr_;

	ftl::Handler<ftl::net::Peer*> connect_cb_;

	static float req_bitrate__;
	static float sample_count__;
	static int64_t last_msg__;
	static MUTEX msg_mtx__;

	std::list<detail::StreamClient> clients_;

	bool _processRequest(ftl::net::Peer &p, ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt);
	void _checkDataRate(size_t tx_size, int64_t tx_latency, int64_t ts);
	bool _sendRequest(ftl::codecs::Channel c, uint8_t frameset, uint8_t frames, uint8_t count, uint8_t bitrate, bool doreset=false);
	void _cleanUp();
};

}
}

#endif  // _FTL_STREAM_NETSTREAM_HPP_
