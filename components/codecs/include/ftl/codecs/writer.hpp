#ifndef _FTL_CODECS_WRITER_HPP_
#define _FTL_CODECS_WRITER_HPP_

#include <iostream>
#include <msgpack.hpp>
//#include <Eigen/Eigen>

#include <ftl/codecs/packet.hpp>

namespace ftl {
namespace codecs {

class Writer {
	public:
	Writer(std::ostream &);
	~Writer();

	bool begin();
	bool write(const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &);
	bool end();

	private:
	std::ostream *stream_;
	msgpack::sbuffer buffer_;
	int64_t timestart_;
};

}
}

#endif  // _FTL_CODECS_WRITER_HPP_
