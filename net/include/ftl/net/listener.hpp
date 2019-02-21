#ifndef _FTL_NET_LISTENER_HPP_
#define _FTL_NET_LISTENER_HPP_

#ifndef WIN32
#include <netinet/in.h>
#endif

#ifdef WIN32
//#include <windows.h>
#include <winsock.h>
#endif

namespace ftl {
namespace net {

class Listener {
	public:
	Listener(const char *uri);
	Listener(int sfd) : descriptor_(sfd) {}
	virtual ~Listener();
	
	bool isListening() { return descriptor_ >= 0; }
	void close();
	int _socket() { return descriptor_; }
	
	private:
	int descriptor_;
	sockaddr_in slocalAddr;
};

};
};

#endif // _FTL_NET_LISTENER_HPP_
