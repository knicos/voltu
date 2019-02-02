#include <ftl/net/raw.hpp>
#include <ftl/uri.hpp>
#include <vector>
#include <iostream>

#ifndef WIN32
#include <errno.h>
#include <fcntl.h>
#endif

#undef ERROR

using ftl::URI;
using ftl::net::raw::Socket;

static std::vector<Socket*> sockets;
static int ssock = INVALID_SOCKET;
static fd_set sfdread;
static fd_set sfderror;
static sockaddr_in slocalAddr;

static int freeSocket() {
	int freeclient = -1;

	//Find a free client slot and allocated it
	for (unsigned int i=0; i<sockets.size(); i++) {
		if (sockets[i] == 0) { // CHECK, was 0 which seems wrong
			freeclient = i;
			break;
		}
	}

	//Max clients reached, so send error
	if (freeclient == -1) {
		if (sockets.size() < ftl::net::raw::MAX_CONNECTIONS) {
			sockets.push_back(0);
			freeclient = sockets.size()-1;
		} else {
			// exceeded max connections
			return -1;
		}
	}

	return freeclient;
}

static int setDescriptors() {
	//Reset all file descriptors
	FD_ZERO(&sfdread);
	FD_ZERO(&sfderror);

	int n = 0;

	//Set file descriptor for the listening socket.
	if (ssock) {
		FD_SET(ssock, &sfdread);
		FD_SET(ssock, &sfderror);
		n = ssock;
	}

	//Set the file descriptors for each client
	for (auto s : sockets) {
		if (s != NULL && s->isConnected()) {
			if (s->_socket() > n) {
				n = s->_socket();
			}

			FD_SET(s->_socket(), &sfdread);
			FD_SET(s->_socket(), &sfderror);
		}
	}

	return n;
}

static int tcpListen(URI &uri) {
	//std::cerr << "TCP Listen: " << uri.getHost() << " : " << uri.getPort() << std::endl;
	#ifdef WIN32
	WSAData wsaData;
	//If Win32 then load winsock
	if (WSAStartup(MAKEWORD(1,1), &wsaData) != 0) {
		return 1;
	}
	#endif

	ssock = socket(AF_INET, SOCK_STREAM, 0);
	if (ssock == INVALID_SOCKET) {
		return 1;
	}

	//Specify listen port and address
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
		return 1;
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
		return 1;
	}
	
	return 0;
}

static int wsListen(URI &uri) {
	return 1;
}

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

int ftl::net::raw::listen(const char *pUri) {
	URI uri(pUri);

	if (uri.getProtocol() == URI::SCHEME_TCP) {
		return tcpListen(uri);
	} else if (uri.getProtocol() == URI::SCHEME_WS) {
		return wsListen(uri);
	} else {
		return 1;
	}
}

void ftl::net::raw::stop() {
	for (auto s : sockets) {
		if (s != NULL) s->close();
	}
	
	sockets.clear();
	
	#ifndef WIN32
	if (ssock != INVALID_SOCKET) close(ssock);
	#else
	if (ssock != INVALID_SOCKET) closesocket(ssock);
	#endif

	ssock = INVALID_SOCKET;
}

Socket *ftl::net::raw::connect(const char *pUri) {
	URI uri(pUri);

	if (uri.getProtocol() == URI::SCHEME_TCP) {
		int csock = tcpConnect(uri);
		Socket *s = new Socket(csock, pUri);
		int fs = freeSocket();
		if (fs >= 0) {
			sockets[fs] = s;
			return s;
		} else {
			return NULL;
		}
	} else if (uri.getProtocol() == URI::SCHEME_WS) {
		wsConnect(uri);
		return NULL;
	} else {
		return NULL;
	}
}

int ftl::net::raw::run(bool blocking) {
	timeval block;
	int n;
	int selres = 1;
	
	//if (ssock == INVALID_SOCKET) return 1;

	bool active = true;
	bool repeat = false;

	while (active || repeat) {
		n = setDescriptors();

		//Wait for a network event or timeout in 3 seconds
		block.tv_sec = (repeat) ? 0 : 3;
		block.tv_usec = 0;
		selres = select(n+1, &sfdread, 0, &sfderror, &block);

		repeat = false;
		active = blocking;

		//Some kind of error occured, it is usually possible to recover from this.
		if (selres <= 0) {
			return 1;
		}

		//If connection request is waiting
		if (FD_ISSET(ssock, &sfdread)) {
			int rsize = sizeof(sockaddr_storage);
			sockaddr_storage addr;
			int freeclient = freeSocket();

			if (freeclient >= 0) {
				// TODO Limit connection rate or allow a pause in accepting
				// TODO Send auto reject message under heavy load

				//Finally accept this client connection.
				int csock = accept(ssock, (sockaddr*)&addr, (socklen_t*)&rsize);

				if (csock != INVALID_SOCKET) {
					Socket *sock = new Socket(csock, NULL);
					sockets[freeclient] = sock;
					
					//Save the ip address
					// deal with both IPv4 and IPv6:
					if (addr.ss_family == AF_INET) {
						struct sockaddr_in *s = (struct sockaddr_in *)&addr;
						//port = ntohs(s->sin_port);
						inet_ntop(AF_INET, &s->sin_addr, sock->m_addr, INET6_ADDRSTRLEN);
					} else { // AF_INET6
						struct sockaddr_in6 *s = (struct sockaddr_in6 *)&addr;
						//port = ntohs(s->sin6_port);
						inet_ntop(AF_INET6, &s->sin6_addr, sock->m_addr, INET6_ADDRSTRLEN);
					}
				}
			}
		}

		//Also check each clients socket to see if any messages or errors are waiting
		for (auto s : sockets) {
			if (s != NULL && s->isConnected()) {
				//If message received from this client then deal with it
				if (FD_ISSET(s->_socket(), &sfdread)) {
					repeat |= s->data();
				//An error occured with this client.
				} else if (FD_ISSET(s->_socket(), &sfderror)) {
					s->error();
				}
			}
		}
	}
	
	return 1;
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

Socket::Socket(int s, const char *uri) : m_uri(uri), m_sock(s), m_pos(0) {
	// Allocate buffer
	m_buffer = new char[BUFFER_SIZE];
}

Socket::~Socket() {
	// Delete socket buffer
	delete [] m_buffer;
}

