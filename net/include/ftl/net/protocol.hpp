#ifndef _FTL_NET_PROTOCOL_HPP_
#define _FTL_NET_PROTOCOL_HPP_

#define FTL_PROTOCOL_HS1		0x0001		// Handshake step 1
#define FTL_PROTOCOL_HS2		0x0002		// Handshake step 2

#define FTL_PROTOCOL_RPC		0x0100
#define FTL_PROTOCOL_RPCRETURN	0x0101

#define FTL_PROTOCOL_P2P		0x1000

namespace ftl {
namespace net {

static const uint32_t MAGIC = 0x23995621;

static const uint8_t PATCH = 0;
static const uint8_t MINOR = 0;
static const uint8_t MAJOR = 1;

inline uint32_t version(int maj, int min, int pat) {
	return (maj << 16) | (min << 8) | pat;
}

inline uint32_t version() {
	return version(MAJOR, MINOR, PATCH);
}

#pragma pack(push,1)

struct Header {
	uint32_t size;
	uint32_t service;
};

struct Handshake {
	uint32_t magic;
	uint32_t version;
	char peerid[16];
};

#pragma pack(pop)



};
};

#endif // _FTL_NET_PROTOCOL_HPP_
