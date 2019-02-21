#include <ftl/uri.hpp>
#include <ftl/net/listener.hpp>
#include <iostream>

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

using namespace ftl;
using ftl::net::Listener;

int tcpListen(URI &uri) {
	int ssock;
	//std::cerr << "TCP Listen: " << uri.getHost() << " : " << uri.getPort() << std::endl;
	#ifdef WIN32
	WSAData wsaData;
	//If Win32 then load winsock
	if (WSAStartup(MAKEWORD(1,1), &wsaData) != 0) {
		return INVALID_SOCKET;
	}
	#endif

	ssock = socket(AF_INET, SOCK_STREAM, 0);
	if (ssock == INVALID_SOCKET) {
		return INVALID_SOCKET;
	}

	//Specify listen port and address
	sockaddr_in slocalAddr;
	slocalAddr.sin_family = AF_INET;
	slocalAddr.sin_addr.s_addr = htonl(INADDR_ANY); // TODO, use that given in URI
	slocalAddr.sin_port = htons(uri.getPort());
	
	int rc = ::bind(ssock, (struct sockaddr*)&slocalAddr, sizeof(slocalAddr));
	
	if (rc == SOCKET_ERROR) {
		#ifndef WIN32
		close(ssock);
		#else
		closesocket(ssock);
		#endif
		ssock = INVALID_SOCKET;
		return INVALID_SOCKET;
	}

	//Attempt to start listening for connection requests.
	rc = ::listen(ssock, 1);

	if (rc == SOCKET_ERROR) {
		#ifndef WIN32
		close(ssock);
		#else
		closesocket(ssock);
		#endif
		ssock = INVALID_SOCKET;
		return INVALID_SOCKET;
	}
	
	return ssock;
}

int wsListen(URI &uri) {
	return INVALID_SOCKET;
}

Listener::Listener(const char *pUri) {
	URI uri(pUri);
	
	descriptor_ = INVALID_SOCKET;

	if (uri.getProtocol() == URI::SCHEME_TCP) {
		descriptor_ = tcpListen(uri);
		std::cout << "Listening: " << pUri << " - " << descriptor_ << std::endl;
	} else if (uri.getProtocol() == URI::SCHEME_WS) {
		descriptor_ = wsListen(uri);
	} else {
		
	}
}

Listener::~Listener() {
	// Close the socket.
	close();
}

void Listener::connection(Socket &s) {
	for (auto h : handler_connect_) h(s);
}

void Listener::close() {
	if (isListening()) {
		#ifndef WIN32
		::close(descriptor_);
		#else
		closesocket(descriptor_);
		#endif
		descriptor_ = INVALID_SOCKET;

		// Attempt auto reconnect?
	}
}

