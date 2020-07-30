#include <ftl/streams/netstream.hpp>

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

#ifndef WIN32
#include <unistd.h>
#include <limits.h>
#endif

using ftl::stream::Net;
using ftl::codecs::StreamPacket;
using ftl::codecs::Packet;
using ftl::codecs::Channel;
using ftl::codecs::codec_t;
using ftl::codecs::definition_t;
using ftl::codecs::kAllFrames;
using ftl::codecs::kAllFramesets;
using std::string;
using std::vector;
using std::optional;

static constexpr int kTallyScale = 10;

float Net::req_bitrate__ = 0.0f;
float Net::sample_count__ = 0.0f;
int64_t Net::last_msg__ = 0;
MUTEX Net::msg_mtx__;

static std::list<std::string> net_streams;
static std::atomic_flag has_bindings = ATOMIC_FLAG_INIT;
static SHARED_MUTEX stream_mutex;

Net::Net(nlohmann::json &config, ftl::net::Universe *net) : Stream(config), active_(false), net_(net), clock_adjust_(0), last_ping_(0) {
	if (!has_bindings.test_and_set()) {
		if (net_->isBound("find_stream")) net_->unbind("find_stream");
		net_->bind("find_stream", [net = net_](const std::string &uri, bool proxy) -> optional<ftl::UUID> {
			LOG(INFO) << "REQUEST FIND STREAM: " << uri;

			ftl::URI u1(uri);
			std::string base = u1.getBaseURI();

			SHARED_LOCK(stream_mutex, lk);
			for (const auto &s : net_streams) {
				ftl::URI u2(s);
				// Don't compare query string components.
				if (base == u2.getBaseURI()) {
					return net->id();
				}
			}
			return {};
		});

		if (net_->isBound("list_streams")) net_->unbind("list_streams");
		net_->bind("list_streams", [this]() {
			LOG(INFO) << "REQUEST LIST STREAMS";
			SHARED_LOCK(stream_mutex, lk);
			//vector<string> streams;
			//streams.push_back(uri_);  // Send full original URI
			return net_streams;
		});
	}

	last_frame_ = 0;
	time_peer_ = ftl::UUID(0);
}

Net::~Net() {
	end();

	// FIXME: Wait to ensure no net callbacks are active.
	// Do something better than this
	std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

bool Net::post(const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
	if (!active_) return false;

	// Check if the channel has been requested recently enough. If not then disable it.
	if (host_ && pkt.data.size() > 0 && spkt.frame_number == 0 && static_cast<int>(spkt.channel) >= 0 && static_cast<int>(spkt.channel) < 32) {
		if (reqtally_[static_cast<int>(spkt.channel)] == 0) {
			--reqtally_[static_cast<int>(spkt.channel)];
			auto sel = selected(0);
			sel -= spkt.channel;
			select(0, sel);
			LOG(INFO) << "Unselect Channel: " << (int)spkt.channel << " (" << (int)spkt.streamID << ")";
		} else {
			--reqtally_[static_cast<int>(spkt.channel)];
		}
	}

	// Lock to prevent clients being added / removed
	{
		SHARED_LOCK(mutex_,lk);
		available(spkt.frameSetID()) += spkt.channel;

		if (host_) {
			auto c = clients_.begin();
			while (c != clients_.end()) {
				auto &client = *c;

				// Quality filter the packets
				if (pkt.bitrate > 0 && pkt.bitrate != client.quality) {
					//++c;
					//LOG(INFO) << "Incorrect quality: " << (int)pkt.bitrate << " but requested " << (int)client.quality;
					//continue;
				}

				try {
					// FIXME: This doesn't work for file sources with file relative timestamps...
					short pre_transmit_latency = short(ftl::timer::get_time() - spkt.localTimestamp);

					if (!net_->send(client.peerid,
							base_uri_,
							pre_transmit_latency,  // Time since timestamp for tx
							spkt,
							pkt)) {

						// Send failed so mark as client stream completed
						client.txcount = client.txmax;
					} else {
						// Count frame as completed only if last block and channel is 0
						if (spkt.streamID == 0 && spkt.frame_number == 0 && spkt.channel == Channel::Colour) ++client.txcount;
					}
				} catch(...) {
					client.txcount = client.txmax;
				}
				++c;
			}
		} else {
			try {
				short pre_transmit_latency = short(ftl::timer::get_time() - spkt.localTimestamp);
				if (!net_->send(peer_,
						base_uri_,
						pre_transmit_latency,  // Time since timestamp for tx
						spkt,
						pkt)) {

				} else {
					// TODO: Some disconnect error
				}
			} catch(...) {
				// TODO: Some disconnect error
			}
		}
	}

	_cleanUp();

	return true;
}

bool Net::begin() {
	if (active_) return true;
	if (!get<string>("uri")) return false;

	uri_ = *get<string>("uri");

	ftl::URI u(uri_);
	if (!u.isValid() || !(u.getScheme() == ftl::URI::SCHEME_FTL)) return false;
	base_uri_ = u.getBaseURI();

	if (net_->isBound(base_uri_)) {
		LOG(ERROR) << "Stream already exists! - " << uri_;
		active_ = false;
		return false;
	}

	net_->bind(base_uri_, [this](ftl::net::Peer &p, short ttimeoff, const ftl::codecs::StreamPacket &spkt_raw, const ftl::codecs::Packet &pkt) {
		int64_t now = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()).time_since_epoch().count();

		if (!active_) return;

		StreamPacket spkt = spkt_raw;
		// FIXME: see #335
		//spkt.timestamp -= clock_adjust_;
		spkt.localTimestamp = now - int64_t(ttimeoff);
		spkt.hint_capability = 0;
		spkt.hint_source_total = 0;
		spkt.version = 4;

		// Manage recuring requests
		if (!host_ && last_frame_ != spkt.timestamp) {
			UNIQUE_LOCK(mutex_, lk);
			if (last_frame_ != spkt.timestamp) {
				last_frame_ = spkt.timestamp;

				if (size() > 0) {
					// TODO: For all framesets
					auto sel = selected(0);

					// A change in channel selections, so send those requests now
					if (sel != last_selected_) {
						auto changed = sel - last_selected_;
						last_selected_ = sel;

						for (auto c : changed) {
							_sendRequest(c, kAllFramesets, kAllFrames, 30, 0);
						}
					}
				}

				// Are we close to reaching the end of our frames request?
				if (tally_ <= 5) {
					// Yes, so send new requests
					if (size() > 0) {
						const auto &sel = selected(0);
						
						for (auto c : sel) {
							_sendRequest(c, kAllFramesets, kAllFrames, 30, 0);
						}
					}
					tally_ = 30*kTallyScale;
				} else {
					--tally_;
				}
			}
		}

		// If hosting and no data then it is a request for data
		// Note: a non host can receive empty data, meaning data is available
		// but that you did not request it
		if (host_ && pkt.data.size() == 0 && (spkt.flags & ftl::codecs::kFlagRequest)) {
			// FIXME: Allow unselecting ...?
			if (spkt.frameSetID() == 255) {
				for (size_t i=0; i<size(); ++i) {
					select(i, selected(i) + spkt.channel);
				}
				if (static_cast<int>(spkt.channel) < 32) {
					reqtally_[static_cast<int>(spkt.channel)] = static_cast<int>(pkt.frame_count)*kTallyScale;
				}
			} else {
				select(spkt.frameSetID(), selected(spkt.frameSetID()) + spkt.channel);
			}

			_processRequest(p, pkt);
		} else {
			// FIXME: Allow availability to change...
			available(spkt.frameSetID()) += spkt.channel;
			//LOG(INFO) << "AVAILABLE: " << (int)spkt.channel;
		}

		//if (cb_) {
			cb_.trigger(spkt, pkt);
			if (pkt.data.size() > 0) _checkDataRate(pkt.data.size(), now-(spkt.timestamp+ttimeoff), spkt.timestamp);
		//}
	});

	// First find non-proxy version, then check for proxy version if no match
	auto p = net_->findOne<ftl::UUID>("find_stream", uri_, false);
	if (!p) p = net_->findOne<ftl::UUID>("find_stream", uri_, true);

	if (!p) {
		LOG(INFO) << "Hosting stream: " << uri_;
		// TODO: Register URI as available.
		host_ = true;

		// Alias the URI to the configurable if not already
		// Allows the URI to be used to get config data.
		if (ftl::config::find(uri_) == nullptr) {
			ftl::config::alias(uri_, this);
		}

		{
			UNIQUE_LOCK(stream_mutex, lk);
			net_streams.push_back(uri_);
		}

		// Automatically set name if missing
		if (!get<std::string>("name")) {
			char hostname[1024] = {0};
			#ifdef WIN32
			DWORD size = 1024;
			GetComputerName(hostname, &size);
			#else
			gethostname(hostname, 1024);
			#endif

			set("name", std::string(hostname));
		}

		active_ = true;
		net_->broadcast("add_stream", uri_);

		return true;
	} else {
		//LOG(INFO) << "Net cfg: " << net_->call<std::string>(*p, "get_cfg", uri_);
	}

	host_ = false;
	peer_ = *p;
	tally_ = 30*kTallyScale;
	for (size_t i=0; i<reqtally_.size(); ++i) reqtally_[i] = 0;

	active_ = true;
	
	// Initially send a colour request just to create the connection
	_sendRequest(Channel::Colour, kAllFramesets, kAllFrames, 30, 0);

	return true;
}

void Net::reset() {
	UNIQUE_LOCK(mutex_, lk);

	if (size() > 0) {
		auto sel = selected(0);
		
		for (auto c : sel) {
			_sendRequest(c, kAllFramesets, kAllFrames, 30, 0);
		}
	}
	tally_ = 30*kTallyScale;
}

bool Net::_sendRequest(Channel c, uint8_t frameset, uint8_t frames, uint8_t count, uint8_t bitrate) {
	if (!active_ || host_) return false;

	//LOG(INFO) << "SENDING REQUEST FOR " << (int)c;

	Packet pkt = {
		codec_t::Any,			// TODO: Allow specific codec requests
		0,
		count,
		bitrate,
		0
	};

	StreamPacket spkt = {
		5,
		ftl::timer::get_time(),
		frameset,
		frames,
		c,
		ftl::codecs::kFlagRequest,
		0,
		0,
		0
	};

	net_->send(peer_, base_uri_, (short)0, spkt, pkt);

	// FIXME: Find a way to use this for correct stream latency info
	if (false) { //if (c == Channel::Colour) {  // TODO: Not every time
		auto start = std::chrono::high_resolution_clock::now();
		//int64_t mastertime;

		try {
			net_->asyncCall<int64_t>(peer_, "__ping__", [this, start](const int64_t &mastertime) {
				auto elapsed = std::chrono::high_resolution_clock::now() - start;
				int64_t latency = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
				clock_adjust_ = ftl::timer::get_time() - mastertime + (latency/2);

				//if (clock_adjust_ > 0) {
					//LOG(INFO) << "LATENCY: " << latency;
				//}
			});
		} catch (...) {
			LOG(ERROR) << "Ping failed";
			// Reset time peer and remove timer
			//time_peer_ = ftl::UUID(0);
			return false;
		}
	}
	return true;
}

void Net::_cleanUp() {
	UNIQUE_LOCK(mutex_,lk);
	for (auto i=clients_.begin(); i!=clients_.end(); ++i) {
		auto &client = *i;
		if (client.txcount >= client.txmax) {
			if (client.peerid == time_peer_) {
				time_peer_ = ftl::UUID(0);
			}
			LOG(INFO) << "Remove peer: " << client.peerid.to_string();
			i = clients_.erase(i);
		}
	}
}

/* Packets for specific framesets, frames and channels are requested in
 * batches (max 255 unique frames by timestamp). Requests are in the form
 * of packets that match the request except the data component is empty.
 */
bool Net::_processRequest(ftl::net::Peer &p, const ftl::codecs::Packet &pkt) {
	{
		UNIQUE_LOCK(mutex_,lk);
		bool found = false;

		// Does the client already exist
		for (auto &c : clients_) {
			if (c.peerid == p.id()) {
				// Yes, so reset internal request counters
				c.txcount = 0;
				c.txmax = static_cast<int>(pkt.frame_count)*kTallyScale;
				found = true;
			}
		}

		// No existing client, so add a new one.
		if (!found) {
			auto &client = clients_.emplace_back();
			client.peerid = p.id();
			client.quality = 255;  // TODO: Use quality given in packet
			client.txcount = 0;
			client.txmax = static_cast<int>(pkt.frame_count)*kTallyScale;

			try {
				connect_cb_.trigger(&p);
			} catch (const ftl::exception &e) {
				LOG(ERROR) << "Exception in stream connect callback: " << e.what();
			}
		}

		// First connected peer (or reconnecting peer) becomes a time server
		/*if (time_peer_ == ftl::UUID(0)) {
			time_peer_ = p.id();
			DLOG(INFO) << "Adding time peer";
		}*/
	}

	return false;
}

void Net::_checkDataRate(size_t tx_size, int64_t tx_latency, int64_t ts) {
	//float actual_mbps = (float(tx_size) * 8.0f * (1000.0f / float(tx_latency))) / 1048576.0f;
    //float min_mbps = (float(tx_size) * 8.0f * (1000.0f / float(ftl::timer::getInterval()))) / 1048576.0f;
    //if (actual_mbps > 0.0f && actual_mbps < min_mbps) LOG(WARNING) << "Bitrate = " << actual_mbps << "Mbps, min required = " << min_mbps << "Mbps";

	UNIQUE_LOCK(msg_mtx__,lk);
	req_bitrate__ += float(tx_size) * 8.0f;
	sample_count__ += 1.0f;

	/*if (ts - last_msg_ >= 1000) {
		DLOG(INFO) << "Required Bitrate = " << (req_bitrate_ / float(ts - last_msg_) * 1000.0f / 1048576.0f) << "Mbps";
		last_msg_ = ts;
		req_bitrate_ = 0.0f;
		sample_count_ = 0.0f;
	}*/
}

float Net::getRequiredBitrate() {
	int64_t ts = ftl::timer::get_time();
	UNIQUE_LOCK(msg_mtx__,lk);
	float r = (req_bitrate__ / float(ts - last_msg__) * 1000.0f / 1048576.0f);
	last_msg__ = ts;
	req_bitrate__ = 0.0f;
	sample_count__ = 0.0f;
	return r;
}

bool Net::end() {
	if (!active_) return false;

	{
		UNIQUE_LOCK(stream_mutex, lk);
		auto i = std::find(net_streams.begin(), net_streams.end(), uri_);
		if (i != net_streams.end()) net_streams.erase(i);
	}

	active_ = false;
	net_->unbind(base_uri_);
	return true;
}

bool Net::active() {
	return active_;
}
