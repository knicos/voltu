#ifndef _FTL_CODECS_PACKET_HPP_
#define _FTL_CODECS_PACKET_HPP_

#include <cstdint>
#include <vector>
#include <ftl/codecs/bitrates.hpp>

#include <msgpack.hpp>

namespace ftl {
namespace codecs {

/**
 * A single network packet for the compressed video stream. It includes the raw
 * data along with any block metadata required to reconstruct. The underlying
 * codec may use its own blocks and packets, in which case this is essentially
 * an empty wrapper around that. It is used in the encoding callback.
 */
struct Packet {
	ftl::codecs::codec_t codec;
	ftl::codecs::definition_t definition;
	uint8_t block_total;	// Packets expected per frame
	uint8_t block_number; 	// This packets number within a frame
	std::vector<uint8_t> data;

	MSGPACK_DEFINE(codec, definition, block_total, block_number, data);
};

/**
 * Add timestamp and channel information to a raw encoded frame packet. This
 * allows the packet to be located within a larger stream and should be sent
 * or included before a frame packet structure.
 */
struct StreamPacket {
	int64_t timestamp;
	uint8_t channel;  // first bit = channel, second bit indicates second channel being sent

	MSGPACK_DEFINE(timestamp, channel);
};

}
}

#endif  // _FTL_CODECS_PACKET_HPP_
