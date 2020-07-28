//#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <loguru.hpp>

#include <ftl/uri.hpp>
#include <ftl/net/listener.hpp>
#include <ftl/net/peer.hpp>
#include <ftl/net/protocol.hpp>
//#include <iostream>

#ifndef WIN32
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define SOCKET int
#endif

#ifdef WIN32
#include <winsock2.h>
#include <windows.h>
typedef int socklen_t;
#endif

using ftl::net::Listener;
using std::shared_ptr;
using ftl::net::Peer;
using ftl::URI;

SOCKET tcpListen(URI &uri) {
	SOCKET ssock;
	//std::cerr << "TCP Listen: " << uri.getHost() << " : " << uri.getPort() << std::endl;
	#ifdef WIN32
	WSAData wsaData;
	//If Win32 then load winsock
	if (WSAStartup(MAKEWORD(1,1), &wsaData) != 0) {
		LOG(ERROR) << "Could not initiate sockets";
		return INVALID_SOCKET;
	}
	#endif

	ssock = socket(AF_INET, SOCK_STREAM, 0);
	if (ssock == INVALID_SOCKET) {
		return INVALID_SOCKET;
	}
	
	int enable = 1;
	if (setsockopt(ssock, SOL_SOCKET, SO_REUSEADDR, (char*)(&enable), sizeof(int)) < 0)
		LOG(ERROR) << "setsockopt(SO_REUSEADDR) failed";

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
		
		LOG(ERROR) << "Could not bind to " << uri.getBaseURI();
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
		
		LOG(ERROR) << "Could not listen on " << uri.getBaseURI();
		return INVALID_SOCKET;
	}
	
	LOG(INFO) << "Listening on " << uri.getBaseURI();
	
	return ssock;
}

SOCKET wsListen(URI &uri) {
	return INVALID_SOCKET;
}

Listener::Listener(const char *pUri) : default_proto_(NULL) {
	URI uri(pUri);
	
	descriptor_ = INVALID_SOCKET;

	if (uri.getProtocol() == URI::SCHEME_TCP) {
		descriptor_ = tcpListen(uri);
		port_ = uri.getPort();
	} else if (uri.getProtocol() == URI::SCHEME_WS) {
		descriptor_ = wsListen(uri);
	} else {
		
	}
}

Listener::~Listener() {
	// Close the socket.
	close();
}

void Listener::connection(shared_ptr<Peer> &s) {
	/*Handshake hs1;
	hs1.magic = ftl::net::MAGIC;
	memset(hs1.id, 0, 16);
	
	if (default_proto_) {
		s->setProtocol(default_proto_);
		hs1.proto_size = default_proto_->id().size();
		s->send(FTL_PROTOCOL_HS1, hs1, default_proto_->id());
	} else {
		s->setProtocol(NULL);
		hs1.proto_size = 0;
		s->send(FTL_PROTOCOL_HS1, hs1);
	}
	
	LOG(INFO) << "Handshake initiated with " << s->getURI();
	for (auto h : handler_connect_) h(s);*/
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

