#pragma once
#ifndef _FTL_RGBD_FILE_SOURCE_HPP_
#define _FTL_RGBD_FILE_SOURCE_HPP_

#include <loguru.hpp>

#include <ftl/rgbd/source.hpp>
#include "player.hpp"
#include <ftl/codecs/decoder.hpp>

#include <list>

namespace ftl {
namespace rgbd {
namespace detail {

class FileSource : public detail::Source {
	public:
	FileSource(ftl::rgbd::Source *, ftl::rgbd::Player *, int sid);
	~FileSource();

	bool capture(int64_t ts);
	bool retrieve();
	bool compute(int n, int b);
	bool isReady();
	void swap();

	//void reset();
	private:
	ftl::rgbd::Player *reader_;
	bool has_calibration_;

	struct PacketPair {
		ftl::codecs::StreamPacket spkt;
		ftl::codecs::Packet pkt;
	};
	
	std::list<PacketPair> cache_[2];
	int cache_read_;
	int cache_write_;
	int sourceid_;

	ftl::codecs::Decoder *decoders_[2];

	bool realtime_;

	void _removeChannel(ftl::codecs::Channel channel);
	void _createDecoder(int ix, const ftl::codecs::Packet &pkt);
};

}
}
}

#endif  // _FTL_RGBD_FILE_SOURCE_HPP_