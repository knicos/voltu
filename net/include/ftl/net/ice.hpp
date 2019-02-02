#ifndef _FTL_NET_ICE_HPP_
#define _FTL_NET_ICE_HPP_

#include <vector>
#include <stdint.h>
#include <string>

namespace ftl {
namespace net {

namespace raw {
	class Socket;
}

namespace ice {

/**
 * Obtain a list of ICE candidate URLs.
 */
int candidates(std::vector<std::string> &c, uint16_t lport, bool tcp);

int stun(std::string &c, uint16_t lport, bool tcp);
int stun(std::string &c, const char *stunuri, uint16_t lport);

/**
 * Request that two sockets connect together directly. This is the main
 * entry point into the ICE mechanism, but should be initiated by the FTL
 * server and not individual nodes.
 */
//int link(ftl::net::raw::Socket *a, ftl::net::raw::Socket *b);

/**
 * Attempt to connect to each candidate in order using the ICE standard.
 */
ftl::net::raw::Socket *connect(std::vector<std::string> &addrs);

} // ice
} // net
} // ftl

#endif // _FTL_NET_ICE_HPP_
