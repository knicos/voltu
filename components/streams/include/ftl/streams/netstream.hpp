#ifndef _FTL_STREAM_NETSTREAM_HPP_
#define _FTL_STREAM_NETSTREAM_HPP_

#include <ftl/config.h>
#include <ftl/net/universe.hpp>
#include <ftl/threads.hpp>
#include <ftl/codecs/packet.hpp>
#include <ftl/streams/stream.hpp>
#include <ftl/handle.hpp>
#include <string>

namespace ftl {
namespace stream {

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
 * Allows network streaming of a number of RGB-Depth sources. Remote machines
 * can discover available streams from an instance of Streamer. It also allows
 * for adaptive bitrate streams where bandwidth can be monitored and different
 * data rates can be requested, it is up to the remote machine to decide on
 * desired bitrate.
 * 
 * The capture and compression steps operate in different threads and each
 * source and bitrate also operate on different threads. For a specific source
 * and specific bitrate there is a single thread that sends data to all
 * requesting clients.
 * 
 * Use ftl::create<Streamer>(parent, name) to construct, don't construct
 * directly.
 * 
 * Note that the streamer attempts to maintain a fixed frame rate by
 * monitoring job processing times and sleeping if there is spare time.
 */
class Net : public Stream {
	public:
	Net(nlohmann::json &config, ftl::net::Universe *net);
	~Net();

	//bool onPacket(const std::function<void(const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &)> &) override;

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
	bool late_;
	int64_t clock_adjust_;
	ftl::UUID time_peer_;
	ftl::UUID peer_;
	int64_t last_frame_;
	int64_t frame_no_;
	int64_t last_ping_;
	std::string uri_;
	std::string base_uri_;
	bool host_;
	int tally_;
	std::array<std::atomic<int>,32> reqtally_;
	std::unordered_set<ftl::codecs::Channel> last_selected_;

	ftl::Handler<ftl::net::Peer*> connect_cb_;

	static float req_bitrate__;
	static float sample_count__;
	static int64_t last_msg__;
	static MUTEX msg_mtx__;

	std::list<detail::StreamClient> clients_;

	//StreamCallback cb_;

	bool _processRequest(ftl::net::Peer &p, const ftl::codecs::Packet &pkt);
	void _checkDataRate(size_t tx_size, int64_t tx_latency, int64_t ts);
	bool _sendRequest(ftl::codecs::Channel c, uint8_t frameset, uint8_t frames, uint8_t count, uint8_t bitrate);
	void _cleanUp();
	
	//void _addClient(int N, int rate, const ftl::UUID &peer, const std::string &dest);
	//void _transmitPacket(const ftl::codecs::Packet &pkt, ftl::codecs::Channel chan, int count);
	//void _transmitPacket(const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt);
};

}
}

#endif  // _FTL_STREAM_NETSTREAM_HPP_
