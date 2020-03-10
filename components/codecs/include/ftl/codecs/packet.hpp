#ifndef _FTL_CODECS_PACKET_HPP_
#define _FTL_CODECS_PACKET_HPP_

#include <cstdint>
#include <vector>
#include <ftl/codecs/codecs.hpp>
#include <ftl/codecs/channels.hpp>
#include <ftl/utility/msgpack.hpp>

namespace ftl {
namespace codecs {

static constexpr uint8_t kAllFrames = 255;
static constexpr uint8_t kAllFramesets = 255;

/**
 * First bytes of our file format.
 */
struct Header {
	const char magic[4] = {'F','T','L','F'};
	uint8_t version = 4;
};

/**
 * Version 2 header padding for potential indexing use.
 */
struct IndexHeader {
	int64_t reserved[8];
};

/**
 * A single network packet for the compressed video stream. It includes the raw
 * data along with any block metadata required to reconstruct. The underlying
 * codec may use its own blocks and packets, in which case this is essentially
 * an empty wrapper around that. It is used in the encoding callback.
 */
struct Packet {
	ftl::codecs::codec_t codec;
	ftl::codecs::definition_t definition;	// Data resolution

	union {
	[[deprecated]] uint8_t block_total;	// v1-3 Packets expected per frame
	uint8_t frame_count;	// v4+ Frames included in this packet
	};

	union {
	[[deprecated]] uint8_t block_number; 	// v1-3 This packets number within a frame
	uint8_t bitrate=0;	// v4+ For multi-bitrate encoding, 0=highest
	};

	uint8_t flags;			// Codec dependent flags (eg. I-Frame or P-Frame)
	std::vector<uint8_t> data;

	MSGPACK_DEFINE(codec, definition, frame_count, bitrate, flags, data);
};

static constexpr unsigned int kStreamCap_Static = 0x01;

/**
 * Add timestamp and channel information to a raw encoded frame packet. This
 * allows the packet to be located within a larger stream and should be sent
 * or included before a frame packet structure.
 */
struct StreamPacket {
	int version;			// FTL version, Not encoded into stream

	int64_t timestamp;
	uint8_t streamID;  		// Source number [or v4 frameset id]

	union {
		[[deprecated]] uint8_t channel_count;	// v1-3 Number of channels to expect for this frame(set) to complete (usually 1 or 2)
		uint8_t frame_number;	// v4+ First frame number (packet may include multiple frames)
	};

	ftl::codecs::Channel channel;		// Actual channel of this current set of packets

	inline int frameNumber() const { return (version >= 4) ? frame_number : streamID; }
	inline size_t frameSetID() const { return (version >= 4) ? streamID : 0; }
	inline int64_t localTimestamp() const { return timestamp + originClockDelta; }

	int64_t originClockDelta;  		// Not message packet / saved
	unsigned int hint_capability;	// Is this a video stream, for example
	size_t hint_source_total;		// Number of tracks per frame to expect

	MSGPACK_DEFINE(timestamp, streamID, frame_number, channel);

	operator std::string() const;
};

/**
 * Combine both packet types into a single packet unit. This pair is always
 * saved or transmitted in a stream together.
 */
struct PacketPair {
	PacketPair(const StreamPacket &s, const Packet &p) : spkt(s), pkt(p) {}
	const StreamPacket &spkt;
	const Packet &pkt;
};

}
}

#endif  // _FTL_CODECS_PACKET_HPP_
