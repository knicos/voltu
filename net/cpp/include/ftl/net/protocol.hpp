#ifndef _FTL_NET_PROTOCOL_HPP_
#define _FTL_NET_PROTOCOL_HPP_

#include <ftl/uuid.hpp>
#include <ftl/config.h>
#include <tuple>

namespace ftl {
namespace net {

typedef std::tuple<uint64_t, uint32_t, ftl::UUID> Handshake;

static const uint64_t kMagic = 0x1099340053640912;
static const uint32_t kVersion = (FTL_VERSION_MAJOR << 16) +
		(FTL_VERSION_MINOR << 8) + FTL_VERSION_PATCH;

};
};

#endif // _FTL_NET_PROTOCOL_HPP_
