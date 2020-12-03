/**
 * @file listener.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#ifndef _FTL_NET_LISTENER_HPP_
#define _FTL_NET_LISTENER_HPP_

#include <ftl/net/common_fwd.hpp>

#include <ftl/net/handlers.hpp>
#include <ftl/net/peer.hpp>

#include <vector>

namespace ftl {
namespace net {

class Protocol;  ///< Unused?

/**
 * Listener for new peer connections on specified port.
 */
class Listener {
	public:
	/**
	 * Use a URI to specify interface address, port and protocol.
	 * 
	 * Example:
	 *  * tcp://localhost:9000
	 */
	explicit Listener(const char *uri);

	/**
	 * Use already provided Socket to listen on.
	 */
	explicit Listener(SOCKET sfd) : descriptor_(sfd), default_proto_(nullptr) {}
	virtual ~Listener();
	
	bool isListening() { return descriptor_ >= 0; }
	void close();
	SOCKET _socket() { return descriptor_; }
	
	/** @deprecated */
	void setProtocol(Protocol *p) { default_proto_ = p; }
	
	/**
	 * Unused
	 */
	void connection(std::shared_ptr<Peer> &s);

	/**
	 * Unused
	 */
	void onConnection(connecthandler_t h) { handler_connect_.push_back(h); };

	inline int port() const { return port_; }
	
	private:
	SOCKET descriptor_;
	Protocol *default_proto_;
	int port_;
	//sockaddr_in slocalAddr;
	std::vector<connecthandler_t> handler_connect_;
};

};
};

#endif // _FTL_NET_LISTENER_HPP_
