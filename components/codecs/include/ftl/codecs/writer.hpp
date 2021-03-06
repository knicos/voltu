/**
 * @file writer.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#ifndef _FTL_CODECS_WRITER_HPP_
#define _FTL_CODECS_WRITER_HPP_

#include <iostream>
#include <ftl/utility/msgpack.hpp>
//#include <Eigen/Eigen>

#include <ftl/threads.hpp>
#include <ftl/codecs/packet.hpp>

namespace ftl {
namespace codecs {

/**
 * Generate FTL file from packet data.
 * 
 * @see ftl::stream::File
 * @deprecated
 */
class Writer {
	public:
	explicit Writer(std::ostream &);
	~Writer();

	bool begin();
	bool write(const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &);
	bool end();
	bool active();

	private:
	std::ostream *stream_;
	msgpack::sbuffer buffer_;
	int64_t timestart_;
	bool active_;
	MUTEX mutex_;
};

}
}

#endif  // _FTL_CODECS_WRITER_HPP_
