#include <ftl/net/universe.hpp>

#ifdef WIN32
#include <Ws2tcpip.h>
#pragma comment(lib, "Rpcrt4.lib")
#endif

using std::string;
using std::vector;
using std::thread;
using ftl::net::Peer;
using ftl::net::Listener;
using ftl::net::Universe;
using nlohmann::json;

Universe::Universe() : active_(true), thread_(Universe::__start, this) {}

Universe::Universe(nlohmann::json &config) :
		active_(true), config_(config), thread_(Universe::__start, this) {
	if (config["listen"].is_array()) {
		for (auto &l : config["listen"]) {
			listen(l);
		}
	} else if (config["listen"].is_string()) {
		listen(config["listen"]);
	}
	
	if (config["peers"].is_array()) {
		for (auto &p : config["peers"]) {
			connect(p);
		}
	}
}

Universe::~Universe() {
	active_ = false;
	thread_.join();
	
	for (auto s : peers_) {
		s->close();
	}
	
	peers_.clear();
	
	for (auto l : listeners_) {
		l->close();
	}
	
	listeners_.clear();
}

bool Universe::listen(const string &addr) {
	auto l = new Listener(addr.c_str());
	if (!l) return false;
	listeners_.push_back(l);
	return l->isListening();
}

bool Universe::connect(const string &addr) {
	auto p = new Peer(addr.c_str(), &disp_);
	if (!p) return false;
	
	if (p->status() != Peer::kInvalid) {
		peers_.push_back(p);
	}
	
	_installBindings(p);
	
	return p->status() == Peer::kConnecting;
}

int Universe::_setDescriptors() {
	//Reset all file descriptors
	FD_ZERO(&sfdread_);
	FD_ZERO(&sfderror_);

	int n = 0;

	//Set file descriptor for the listening sockets.
	for (auto l : listeners_) {
		if (l != nullptr && l->isListening()) {
			FD_SET(l->_socket(), &sfdread_);
			FD_SET(l->_socket(), &sfderror_);
			if (l->_socket() > n) n = l->_socket();
		}
	}

	//Set the file descriptors for each client
	for (auto s : peers_) {
		if (s != nullptr && s->isValid()) {
			
			if (s->_socket() > n) {
				n = s->_socket();
			}

			if (s->isWaiting()) {
				FD_SET(s->_socket(), &sfdread_);
			}
			FD_SET(s->_socket(), &sfderror_);
		}
	}

	return n;
}

void Universe::_installBindings(Peer *p) {
	p->bind("__subscribe__", [this](const string &uri) {
		// Add this peer to subscription list for uri resource
	});
	
	
}

void Universe::__start(Universe * u) {
	u->_run();
}

void Universe::_run() {
	timeval block;

	while (active_) {
		int n = _setDescriptors();
		int selres = 1;

		//Wait for a network event or timeout in 3 seconds
		block.tv_sec = 0;
		block.tv_usec = 10000;
		selres = select(n+1, &sfdread_, 0, &sfderror_, &block);

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
		for (auto l : listeners_) {
			if (l && l->isListening()) {
				if (FD_ISSET(l->_socket(), &sfdread_)) {
					int rsize = sizeof(sockaddr_storage);
					sockaddr_storage addr;

					//int freeclient = freeSocket();

					//if (freeclient >= 0) {
						// TODO Limit connection rate or allow a pause in accepting
						// TODO Send auto reject message under heavy load

						//Finally accept this client connection.
						int csock = accept(l->_socket(), (sockaddr*)&addr, (socklen_t*)&rsize);

						if (csock != INVALID_SOCKET) {
							auto p = new Peer(csock, &disp_);
							peers_.push_back(p);
							
							_installBindings(p);
						}
					//}
				}
			}
		}

		//Also check each clients socket to see if any messages or errors are waiting
		for (auto s : peers_) {
			if (s != NULL && s->isValid()) {
				//If message received from this client then deal with it
				if (FD_ISSET(s->_socket(), &sfdread_)) {
					//s->data();
					//std::cout << "QUEUE DATA PROC" << std::endl;
					//p.push([](int id, Peer *s) {
					//	std::cout << "Processing in thread " << std::to_string(id) << std::endl;
						s->data();
					//}, s);
				}
				if (FD_ISSET(s->_socket(), &sfderror_)) {
					s->socketError();
				}
			} else if (s != NULL) {
				// Erase it
				
				for (auto i=peers_.begin(); i!=peers_.end(); i++) {
					if ((*i) == s) {
						LOG(INFO) << "REMOVING SOCKET";
						peers_.erase(i); break;
					}
				}
			}
		}
	}
}

