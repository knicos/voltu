#include <ftl/net.hpp>
#include <ftl/net/listener.hpp>
#include <ftl/net/socket.hpp>

#ifdef WIN32
#include <Ws2tcpip.h>
#endif

#include <vector>
#include <iostream>
#include <chrono>

using namespace std;
using namespace std::chrono;
using ftl::net::Listener;
using ftl::net::Socket;

 std::vector<shared_ptr<ftl::net::Socket>> sockets;
static std::vector<shared_ptr<ftl::net::Listener>> listeners;
static fd_set sfdread;
static fd_set sfderror;

/*static int freeSocket() {
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
}*/

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
		if (s != nullptr && s->isValid()) {
			
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
	shared_ptr<Socket> s(new Socket((uri == NULL) ? "" : uri));
	sockets.push_back(s);
	return s;
}

void ftl::net::stop() {
	for (auto s : sockets) {
		s->close();
	}
	
	sockets.clear();
	
	for (auto l : listeners) {
		l->close();
	}
	
	listeners.clear();
}

bool _run(bool blocking, bool nodelay) {
	timeval block;
	
	//if (ssock == INVALID_SOCKET) return 1;

	bool active = true;
	bool repeat = nodelay;

	while (active || repeat) {
		int n = setDescriptors();
		int selres = 1;

		//Wait for a network event or timeout in 3 seconds
		block.tv_sec = (repeat) ? 0 : 3;
		block.tv_usec = 0;
		selres = select(n+1, &sfdread, 0, &sfderror, &block);

		repeat = false;
		active = blocking;

		//Some kind of error occured, it is usually possible to recover from this.
		if (selres < 0) {
			std::cout << "SELECT ERROR " << selres << std::endl;
			//return false;
			continue;
		} else if (selres == 0) {
			// Timeout, nothing to do...
			continue;
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
							auto sock = make_shared<Socket>(csock);
							sockets.push_back(sock);
							
							// Call connection handlers
							l->connection(sock);
						}
					//}
				}
			}
		}

		//Also check each clients socket to see if any messages or errors are waiting
		for (auto s : sockets) {
			if (s != NULL && s->isValid()) {
				//If message received from this client then deal with it
				if (FD_ISSET(s->_socket(), &sfdread)) {
					repeat |= s->data();
				}
				if (FD_ISSET(s->_socket(), &sfderror)) {
					s->error();
				}
			} else if (s != NULL) {
				// Erase it
				
				for (auto i=sockets.begin(); i!=sockets.end(); i++) {
					if ((*i) == s) {
						std::cout << "REMOVING SOCKET" << std::endl;
						sockets.erase(i); break;
					}
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

void ftl::net::wait(std::function<bool(void)> f, float to) {
	auto start = steady_clock::now();
	while (!f() && duration<float>(steady_clock::now() - start).count() < to)
		_run(false,false);
}

bool ftl::net::run(bool async) {
	if (async) {
		// TODO Start thread
	} else {
		return _run(true,false);
	}
	
	return false;
}

