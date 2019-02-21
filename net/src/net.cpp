#include <ftl/net.hpp>
#include <ftl/net/listener.hpp>
#include <ftl/net/socket.hpp>

#include <vector>
#include <iostream>

using namespace std;
using ftl::net::Listener;
using ftl::net::Socket;

static std::vector<shared_ptr<ftl::net::Socket>> sockets;
static std::vector<shared_ptr<ftl::net::Listener>> listeners;
static fd_set sfdread;
static fd_set sfderror;

static int freeSocket() {
	int freeclient = -1;

	//Find a free client slot and allocated it
	for (unsigned int i=0; i<sockets.size(); i++) {
		if (sockets[i] == nullptr) { // CHECK, was 0 which seems wrong
			freeclient = i;
			break;
		}
	}

	//Max clients reached, so send error
	if (freeclient == -1) {
		if (sockets.size() < ftl::net::MAX_CONNECTIONS) {
			sockets.push_back(shared_ptr<Socket>(nullptr));
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

	//Set file descriptor for the listening sockets.
	for (auto l : listeners) {
		if (l != nullptr && l->isListening()) {
			FD_SET(l->_socket(), &sfdread);
			FD_SET(l->_socket(), &sfderror);
			if (l->_socket() > n) n = l->_socket();
		}
	}

	//Set the file descriptors for each client
	for (auto s : sockets) {
		if (s != nullptr && s->isConnected()) {
			
			if (s->_socket() > n) {
				n = s->_socket();
			}

			FD_SET(s->_socket(), &sfdread);
			FD_SET(s->_socket(), &sfderror);
		}
	}

	return n;
}

shared_ptr<Listener> ftl::net::listen(const char *uri) {
	shared_ptr<Listener> l(new Listener(uri));
	listeners.push_back(l);
	return l;
}

shared_ptr<Socket> ftl::net::connect(const char *uri) {
	shared_ptr<Socket> s(new Socket(uri));
	int fs = freeSocket();
	if (fs >= 0) {
		sockets[fs] = s;
		return s;
	} else {
		return NULL;
	}
}

void ftl::net::stop() {
	for (auto s : sockets) {
		if (s != NULL) s->close();
	}
	
	sockets.clear();
	
	/*#ifndef WIN32
	if (ssock != INVALID_SOCKET) close(ssock);
	#else
	if (ssock != INVALID_SOCKET) closesocket(ssock);
	#endif

	ssock = INVALID_SOCKET;*/
	
	for (auto l : listeners) {
		l->close();
	}
	
	listeners.clear();
}

bool _run(bool blocking, bool nodelay) {
	timeval block;
	int n;
	int selres = 1;
	
	//if (ssock == INVALID_SOCKET) return 1;

	bool active = true;
	bool repeat = nodelay;

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
			return false;
		}

		//If connection request is waiting
		for (auto l : listeners) {
			if (l && l->isListening()) {
				if (FD_ISSET(l->_socket(), &sfdread)) {
					int rsize = sizeof(sockaddr_storage);
					sockaddr_storage addr;
					//int freeclient = freeSocket();

					//if (freeclient >= 0) {
						// TODO Limit connection rate or allow a pause in accepting
						// TODO Send auto reject message under heavy load

						//Finally accept this client connection.
						int csock = accept(l->_socket(), (sockaddr*)&addr, (socklen_t*)&rsize);

						if (csock != INVALID_SOCKET) {
							shared_ptr<Socket> sock(new Socket(csock));
							//sockets[freeclient] = sock;
							
							sockets.push_back(sock);
							
							// TODO Save the ip address
							// deal with both IPv4 and IPv6:
							/*if (addr.ss_family == AF_INET) {
								struct sockaddr_in *s = (struct sockaddr_in *)&addr;
								//port = ntohs(s->sin_port);
								inet_ntop(AF_INET, &s->sin_addr, sock->m_addr, INET6_ADDRSTRLEN);
							} else { // AF_INET6
								struct sockaddr_in6 *s = (struct sockaddr_in6 *)&addr;
								//port = ntohs(s->sin6_port);
								inet_ntop(AF_INET6, &s->sin6_addr, sock->m_addr, INET6_ADDRSTRLEN);
							}*/
						}
					//}
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
	
	return true;
}

bool ftl::net::check() {
	return _run(false,true);
}

bool ftl::net::wait() {
	return _run(false,false);
}

bool ftl::net::run(bool async) {
	if (async) {
		// TODO Start thread
	} else {
		return _run(true,false);
	}
	
	return false;
}

