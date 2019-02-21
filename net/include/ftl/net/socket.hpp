#ifndef _FTL_NET_SOCKET_HPP_
#define _FTL_NET_SOCKET_HPP_

#include <ftl/net.hpp>
#include <ftl/net/handlers.hpp>

#ifndef WIN32
#define INVALID_SOCKET -1
#include <netinet/in.h>
#endif

#ifdef WIN32
//#include <windows.h>
#include <winsock.h>
#endif

namespace ftl {
namespace net {

class Socket {
	public:
	Socket(const char *uri);
	Socket(int s);
	~Socket();
	
	int close();

	int send(uint32_t service, std::string &data);
	int send(uint32_t service, std::ostringstream &data);
	int send(uint32_t service, void *data, int length);

	//friend bool ftl::net::run(bool);

	int _socket() { return m_sock; };

	bool isConnected() { return m_sock != INVALID_SOCKET; };
	bool isValid() { return m_valid; };

	void onMessage(sockdatahandler_t handler) { m_handler = handler; }
	void onError(sockerrorhandler_t handler) {}
	void onConnect(sockconnecthandler_t handler) {}
	void onDisconnect(sockdisconnecthandler_t handler) {}
	
	bool data();
	void error();

	protected:

	char m_addr[INET6_ADDRSTRLEN];

	private:
	const char *m_uri;
	int m_sock;
	size_t m_pos;
	char *m_buffer;
	sockdatahandler_t m_handler;
	bool m_valid;

	static const int MAX_MESSAGE = 10*1024*1024; // 10Mb currently
	static const int BUFFER_SIZE = MAX_MESSAGE + 16;
};

};
};

#endif // _FTL_NET_SOCKET_HPP_
