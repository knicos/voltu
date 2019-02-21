#include <ftl/uri.hpp>
#include <ftl/net/socket.hpp>

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

#include <iostream>

using namespace ftl;
using ftl::net::Socket;
using namespace std;

static int tcpConnect(URI &uri) {
	int rc;
	sockaddr_in destAddr;

	//std::cerr << "TCP Connect: " << uri.getHost() << " : " << uri.getPort() << std::endl;

	#ifdef WIN32
	WSAData wsaData;
	if (WSAStartup(MAKEWORD(1,1), &wsaData) != 0) {
		//ERROR
		return INVALID_SOCKET;
	}
	#endif
	
	//We want a TCP socket
	int csocket = socket(AF_INET, SOCK_STREAM, 0);

	if (csocket == INVALID_SOCKET) {
		return INVALID_SOCKET;
	}

	#ifdef WIN32
	HOSTENT *host = gethostbyname(uri.getHost().c_str());
	#else
	hostent *host = gethostbyname(uri.getHost().c_str());
	#endif

	if (host == NULL) {
		#ifndef WIN32
		close(csocket);
		#else
		closesocket(csocket);
		#endif

		std::cerr << "Address not found : " << uri.getHost() << std::endl;

		return INVALID_SOCKET;
	}

	destAddr.sin_family = AF_INET;
	destAddr.sin_addr.s_addr = ((in_addr *)(host->h_addr))->s_addr;
	destAddr.sin_port = htons(uri.getPort());

	// Make nonblocking
	/*long arg = fcntl(csocket, F_GETFL, NULL));
	arg |= O_NONBLOCK;
	fcntl(csocket, F_SETFL, arg) < 0)*/
	
	rc = ::connect(csocket, (struct sockaddr*)&destAddr, sizeof(destAddr));

	if (rc < 0) {
		if (errno == EINPROGRESS) {

		} else {
			#ifndef WIN32
			close(csocket);
			#else
			closesocket(csocket);
			#endif

			std::cerr << "Could not connect" << std::endl;

			return INVALID_SOCKET;
		}
	}

	// Make blocking again
	/*rg = fcntl(csocket, F_GETFL, NULL));
	arg &= (~O_NONBLOCK);
	fcntl(csocket, F_SETFL, arg) < 0)*/
	
	// Handshake??

	return csocket;
}

static int wsConnect(URI &uri) {
	return 1;
}

Socket::Socket(int s) : m_sock(s), m_pos(0) {
	// TODO Get the remote address.
	m_valid = true;
}

Socket::Socket(const char *pUri) : m_uri(pUri), m_pos(0) {
	// Allocate buffer
	m_buffer = new char[BUFFER_SIZE];
	
	URI uri(pUri);
	
	m_valid = false;
	m_sock = INVALID_SOCKET;

	if (uri.getProtocol() == URI::SCHEME_TCP) {
		m_sock = tcpConnect(uri);
		m_valid = true;
	} else if (uri.getProtocol() == URI::SCHEME_WS) {
		wsConnect(uri);
	} else {
	}
}

int Socket::close() {
	if (isConnected()) {
		#ifndef WIN32
		::close(m_sock);
		#else
		closesocket(m_sock);
		#endif
		m_sock = INVALID_SOCKET;

		// Attempt auto reconnect?
	}
	return 0;
}

void Socket::error() {
	int err;
	uint32_t optlen = sizeof(err);
	getsockopt(m_sock, SOL_SOCKET, SO_ERROR, &err, &optlen);

	std::cerr << "GOT A SOCKET ERROR : " << err << std::endl;
	//close();
}

bool Socket::data() {
	//std::cerr << "GOT SOCKET DATA" << std::endl;

	//Read data from socket
	size_t n = 0;
	uint32_t len = 0;

	if (m_pos < 4) {
		n = 4 - m_pos;
	} else {
		len = *(int*)m_buffer;
		n = len+4-m_pos;
	}

	while (m_pos < len+4) {
		if (len > MAX_MESSAGE) {
			close();
			return false; // Prevent DoS
		}

		const int rc = recv(m_sock, m_buffer+m_pos, n, 0);

		if (rc > 0) {
			m_pos += static_cast<size_t>(rc);

			if (m_pos < 4) {
				n = 4 - m_pos;
			} else {
				len = *(int*)m_buffer;
				n = len+4-m_pos;
			}
		} else if (rc == EWOULDBLOCK || rc == 0) {
			// Data not yet available
			return false;
		} else {
			// Close socket due to error
			close();
			return false;
		}
	}

	// All data available
	if (m_handler) {
		uint32_t service = ((uint32_t*)m_buffer)[1];
		auto d = std::string(m_buffer+8, len-4);
		//std::cerr << "DATA : " << service << " -> " << d << std::endl;
		m_handler(service, d); 
	}

	m_pos = 0;

	return true;
}

Socket::~Socket() {
	close();
	
	// Delete socket buffer
	delete [] m_buffer;
}

