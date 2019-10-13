#ifndef _FTL_CODECS_PACKET_HPP_
#define _FTL_CODECS_PACKET_HPP_

#include <cstdint>
#include <vector>
#include <ftl/codecs/bitrates.hpp>
#include <ftl/codecs/channels.hpp>

#include <msgpack.hpp>

namespace ftl {
namespace codecs {

/**
 * First bytes of our file format.
 */
struct Header {
	const char magic[4] = {'F','T','L','F'};
	uint8_t version = 1;
};

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
	uint8_t flags;			// Codec dependent flags (eg. I-Frame or P-Frame)
	std::vector<uint8_t> data;

	MSGPACK_DEFINE(codec, definition, block_total, block_number, flags, data);
};

/**
 * Add timestamp and channel information to a raw encoded frame packet. This
 * allows the packet to be located within a larger stream and should be sent
 * or included before a frame packet structure.
 */
struct StreamPacket {
	int64_t timestamp;
	uint8_t streamID;  		// Source number...
	uint8_t channel_count;	// Number of channels to expect for this frame to complete (usually 1 or 2)
	ftl::codecs::Channel channel;		// Actual channel of this current set of packets

	MSGPACK_DEFINE(timestamp, streamID, channel_count, channel);
};

}
}

#endif  // _FTL_CODECS_PACKET_HPP_
