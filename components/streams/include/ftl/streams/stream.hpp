#ifndef _FTL_STREAM_STREAM_HPP_
#define _FTL_STREAM_STREAM_HPP_

#include <loguru.hpp>
#include <ftl/configuration.hpp>
#include <ftl/configurable.hpp>
#include <ftl/rgbd/source.hpp>
#include <ftl/rgbd/group.hpp>
#include <ftl/net/universe.hpp>
#include <ftl/codecs/encoder.hpp>
#include <ftl/threads.hpp>
#include <string>
#include <vector>
#include <map>
#include <atomic>

namespace ftl {
namespace stream {

typedef std::function<void(const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &)> StreamCallback;

/**
 * Base stream class to be implemented. Provides encode and decode functionality
 * around a generic packet read and write mechanism. Some specialisations will
 * provide and automatically handle control signals.
 * 
 * Streams are bidirectional, frames can be both received and written.
 */
class Stream : public ftl::Configurable {
	public:
	explicit Stream(nlohmann::json &config) : ftl::Configurable(config) {};
	virtual ~Stream() {};

	/**
	 * Obtain all packets for next frame. The provided callback function is
	 * called once for every packet. This function might continue to call the
	 * callback even after the read function returns, for example with a
	 * NetStream.
	 */
	virtual bool onPacket(const StreamCallback &)=0;

	virtual bool post(const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &)=0;

	/**
	 * Start the stream. Calls to the onPacket callback will only occur after
	 * a call to this function (and before a call to end()).
	 */
	virtual bool begin()=0;

	virtual bool end()=0;

	/**
	 * Is the stream active? Generally true if begin() has been called, false
	 * initially and after end(). However, it may go false for other reasons.
	 * If false, no calls to onPacket will occur and posts will be ignored.
	 */
	virtual bool active()=0;

	/**
	 * Perform a forced reset of the stream. This means something different
	 * depending on stream type, for example with a network stream it involves
	 * resending all stream requests as if a reconnection had occured.
	 */
	virtual void reset();

	/**
	 * Query available video channels for a frameset.
	 */
	const ftl::codecs::Channels<0> &available(int fs) const;

	/**
	 * Query selected channels for a frameset. Channels not selected may not
	 * be transmitted, received or decoded.
	 */
	const ftl::codecs::Channels<0> &selected(int fs) const;

	/**
	 * Change the video channel selection for a frameset.
	 */
	void select(int fs, const ftl::codecs::Channels<0> &, bool make=false);

	/**
	 * Number of framesets in stream.
	 */
	inline size_t size() const { return state_.size(); }

	protected:

	/**
	 * Allow modification of available channels. Calling this with an invalid
	 * fs number will create that frameset and increase the size.
	 */
	ftl::codecs::Channels<0> &available(int fs);

	private:
	struct FSState {
		ftl::codecs::Channels<0> selected;
		ftl::codecs::Channels<0> available;
	};

	std::vector<FSState> state_;
	mutable SHARED_MUTEX mtx_;
};

/**
 * Combine multiple streams into a single stream. StreamPackets are modified
 * by mapping the stream identifiers consistently to new values. Both reading
 * and writing are supported but a write must be preceeded by a read for the
 * stream mapping to be registered.
 */
class Muxer : public Stream {
	public:
	explicit Muxer(nlohmann::json &config);
	virtual ~Muxer();

	void add(Stream *);

	bool onPacket(const StreamCallback &) override;

	bool post(const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &) override;

	bool begin() override;
	bool end() override;
	bool active() override;

	void reset() override;

	private:
	struct StreamEntry {
		Stream *stream;
		std::vector<int> maps;
	};

	std::vector<StreamEntry> streams_;
	std::vector<std::pair<int,int>> revmap_;
	int nid_;
	StreamCallback cb_;
	SHARED_MUTEX mutex_;

	void _notify(const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt);
	int _lookup(int sid, int ssid);
};

/**
 * Forward all data to all child streams. Unlike the muxer which remaps the
 * stream identifiers in the stream packet, this does not alter the stream
 * packets.
 */
class Broadcast : public Stream {
	public:
	explicit Broadcast(nlohmann::json &config);
	virtual ~Broadcast();

	void add(Stream *);

	bool onPacket(const std::function<void(const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &)> &) override;

	bool post(const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &) override;

	bool begin() override;
	bool end() override;
	bool active() override;

	void reset() override;

	private:
	std::list<Stream*> streams_;
	StreamCallback cb_;
	SHARED_MUTEX mutex_;
};

/**
 * Allow packet interception by a callback between two other streams.
 */
class Intercept : public Stream {
	public:
	explicit Intercept(nlohmann::json &config);
	virtual ~Intercept();

	void setStream(Stream *);

	bool onPacket(const StreamCallback &) override;
	bool onIntercept(const StreamCallback &);

	bool post(const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &) override;

	bool begin() override;
	bool end() override;
	bool active() override;

	void reset() override;

	private:
	Stream *stream_;
	StreamCallback cb_;
	StreamCallback intercept_;
	SHARED_MUTEX mutex_;
};

}
}

#endif  // _FTL_STREAM_STREAM_HPP_
