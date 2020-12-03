/**
 * @file reader.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#ifndef _FTL_CODECS_READER_HPP_
#define _FTL_CODECS_READER_HPP_

#include <iostream>
#include <inttypes.h>
#include <functional>

#include <ftl/utility/msgpack.hpp>
#include <ftl/codecs/packet.hpp>
#include <ftl/threads.hpp>

namespace ftl {
namespace codecs {

/**
 * FTL file reader. Deprecated so use ftl::stream::File instead. It reads
 * `StreamPacket` and `Packet` structures sequentially from the file.
 * 
 * @see ftl::stream::File
 * @deprecated
 */
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
	 * 
	 * @param ts Milliseconds timestamp, read all packets up to this point
	 * @param cb Callback for each read packet.
	 * @return true if there are more packets to read.
	 */
	bool read(int64_t ts, const std::function<void(const ftl::codecs::StreamPacket &, ftl::codecs::Packet &)> &cb);

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
	inline int version() const { return version_; }

	private:
	std::istream *stream_;
	msgpack::unpacker buffer_;
	std::list<std::tuple<StreamPacket,Packet>> data_;
	bool has_data_;
	int64_t timestart_;
	bool playing_;
	int version_;

	MUTEX mtx_;

	std::vector<std::function<void(const ftl::codecs::StreamPacket &, ftl::codecs::Packet &)>> handlers_;
};

}  // namespace codecs
}  // namespace ftl

#endif  // _FTL_CODECS_READER_HPP_
