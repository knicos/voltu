#ifndef _FTL_NET_HPP_
#define _FTL_NET_HPP_

#include "ftl/net/raw.hpp"

namespace ftl {
namespace net {

raw::Socket *connect(const char *uri) { return raw::connect(uri); }

}
}

#endif // _FTL_NET_HPP_
