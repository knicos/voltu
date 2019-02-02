#ifndef _FTL_NET_HPP_
#define _FTL_NET_HPP_

#include <functional>
#include <sstream>
#include <string>

#ifndef WIN32
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#endif

#ifdef WIN32
#include <windows.h>
#include <winsock.h>
typedef int socklen_t;
#define MSG_WAITALL 0
#endif

namespace ftl {
namespace net {
namespace raw {

class Socket;

int listen(const char *uri);
void stop();
int run(bool blocking);

/**
 * Accepts tcp, ipc and ws URIs. An example would be:
 *  ws://ftl.utu.fi/api/connect
 */
Socket *connect(const char *uri);

typedef std::function<void(int, std::string&)> sockdatahandler_t;
typedef std::function<void(int)> sockerrorhandler_t;
typedef std::function<void()> sockconnecthandler_t;
typedef std::function<void(int)> sockdisconnecthandler_t;

typedef std::function<void(Socket&, int, std::string&)> datahandler_t;
typedef std::function<void(Socket&, int)> errorhandler_t;
typedef std::function<void(Socket&)> connecthandler_t;
typedef std::function<void(Socket&)> disconnecthandler_t;

class Socket {
	public:
	int close();

	int send(uint32_t service, std::string &data);
	int send(uint32_t service, std::ostringstream &data);
	int send(uint32_t service, void *data, int length);

	friend int ftl::net::raw::listen(const char*);
	friend Socket *ftl::net::raw::connect(const char*);
	friend int ftl::net::raw::run(bool);

	int _socket() { return m_sock; };

	bool isConnected() { return m_sock != INVALID_SOCKET; };

	void onMessage(sockdatahandler_t handler) { m_handler = handler; }
	void onError(sockerrorhandler_t handler) {}
	void onConnect(sockconnecthandler_t handler) {}
	void onDisconnect(sockdisconnecthandler_t handler) {}

	protected:
	Socket(int s, const char *uri);
	~Socket();

	bool data();
	void error();

	char m_addr[INET6_ADDRSTRLEN];

	private:
	const char *m_uri;
	int m_sock;
	size_t m_pos;
	char *m_buffer;
	sockdatahandler_t m_handler;

	static const int MAX_MESSAGE = 10*1024*1024; // 10Mb currently
	static const int BUFFER_SIZE = MAX_MESSAGE + 16;
};

/**
 * Get the number of current connections.
 * @return Connection count
 */
int connections();

void onMessage(datahandler_t handler);
void onConnect(connecthandler_t handler);
void onDisconnect(disconnecthandler_t handler);
void onError(errorhandler_t handler);

const int MAX_CONNECTIONS = 100; // TODO Is this a good number?

} // raw
} // net
} // ftl

#endif // _FTL_NET_HPP_
