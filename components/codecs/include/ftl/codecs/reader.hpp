#ifndef _FTL_CODECS_READER_HPP_
#define _FTL_CODECS_READER_HPP_

#include <iostream>
#include <msgpack.hpp>
#include <inttypes.h>
#include <functional>

#include <ftl/codecs/packet.hpp>
#include <ftl/threads.hpp>

namespace ftl {
namespace codecs {

class Reader {
	public:
	explicit Reader(std::istream &);
	~Reader();

	/**
	 * Read packets up to and including requested timestamp. A provided callback
	 * is called for each packet read, in order stored in file. Returns true if
	 * there are still more packets available beyond specified timestamp, false
	 * otherwise (end-of-file). Timestamps are in local (clock adjusted) time
	 * and the timestamps stored in the file are aligned to the time when open
	 * was called.
	 */
	bool read(int64_t ts, const std::function<void(const ftl::codecs::StreamPacket &, ftl::codecs::Packet &)> &);

	/**
	 * An alternative version of read where packet events are generated for
	 * specific streams, allowing different handlers for different streams.
	 * This allows demuxing and is used by player sources. Each source can call
	 * this read, only the first one will generate the data packets.
	 */
	bool read(int64_t ts);

	void onPacket(int streamID, const std::function<void(const ftl::codecs::StreamPacket &, ftl::codecs::Packet &)> &);

	bool begin();
	bool end();

	inline int64_t getStartTime() const { return timestart_; };

	private:
	std::istream *stream_;
	msgpack::unpacker buffer_;
	std::tuple<StreamPacket,Packet> data_;
	bool has_data_;
	int64_t timestart_;
	bool playing_;

	MUTEX mtx_;

	std::vector<std::function<void(const ftl::codecs::StreamPacket &, ftl::codecs::Packet &)>> handlers_;
};

}
}

#endif  // _FTL_CODECS_READER_HPP_
