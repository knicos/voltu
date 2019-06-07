#ifndef _FTL_NET_LISTENER_HPP_
#define _FTL_NET_LISTENER_HPP_

#include <ftl/net/common.hpp>

#include <ftl/net/handlers.hpp>
#include <ftl/net/peer.hpp>

#include <vector>

namespace ftl {
namespace net {

class Protocol;

class Listener {
	public:
	explicit Listener(const char *uri);
	explicit Listener(SOCKET sfd) : descriptor_(sfd), default_proto_(nullptr) {}
	virtual ~Listener();
	
	bool isListening() { return descriptor_ >= 0; }
	void close();
	SOCKET _socket() { return descriptor_; }
	
	void setProtocol(Protocol *p) { default_proto_ = p; }
	
	void connection(std::shared_ptr<Peer> &s);
	void onConnection(connecthandler_t h) { handler_connect_.push_back(h); };
	
	private:
	SOCKET descriptor_;
	Protocol *default_proto_;
	sockaddr_in slocalAddr;
	std::vector<connecthandler_t> handler_connect_;
};

};
};

#endif // _FTL_NET_LISTENER_HPP_
