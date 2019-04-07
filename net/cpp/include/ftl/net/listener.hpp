#ifndef _FTL_NET_LISTENER_HPP_
#define _FTL_NET_LISTENER_HPP_

#ifndef WIN32
#include <netinet/in.h>
#endif

#ifdef WIN32
//#include <windows.h>
#include <winsock2.h>
#endif

#include <ftl/net/handlers.hpp>
#include <ftl/net/peer.hpp>

#include <vector>

namespace ftl {
namespace net {

class Protocol;

class Listener {
	public:
	explicit Listener(const char *uri);
	explicit Listener(int sfd) : descriptor_(sfd), default_proto_(nullptr) {}
	virtual ~Listener();
	
	bool isListening() { return descriptor_ >= 0; }
	void close();
	int _socket() { return descriptor_; }
	
	void setProtocol(Protocol *p) { default_proto_ = p; }
	
	void connection(std::shared_ptr<Peer> &s);
	void onConnection(connecthandler_t h) { handler_connect_.push_back(h); };
	
	private:
	int descriptor_;
	Protocol *default_proto_;
	sockaddr_in slocalAddr;
	std::vector<connecthandler_t> handler_connect_;
};

};
};

#endif // _FTL_NET_LISTENER_HPP_
