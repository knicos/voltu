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
using ftl::UUID;
using std::optional;

Universe::Universe() : active_(true), thread_(Universe::__start, this) {
	_installBindings();
}

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
	
	_installBindings();
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
	
	p->onConnect([this](Peer &p) {
		peer_ids_[p.id()] = &p;
	});
	
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

}

void Universe::_installBindings() {
	bind("__subscribe__", [this](const UUID &id, const string &uri) -> bool {
		LOG(INFO) << "Subscription to " << uri << " by " << id.to_string();
		subscribers_[ftl::URI(uri).to_string()].push_back(id);
		return true;
	});
	
	bind("__owner__", [this](const std::string &res) -> optional<UUID> {
		LOG(INFO) << "SOMEONE ASKS FOR " << res;
		if (owned_.count(res) > 0) return ftl::net::this_peer;
		else return {};
	});
}

Peer *Universe::getPeer(const UUID &id) const {
	auto ix = peer_ids_.find(id);
	if (ix == peer_ids_.end()) return nullptr;
	else return ix->second;
}

optional<UUID> Universe::findOwner(const string &res) {
	// TODO(nick) cache this information
	return findOne<UUID>("__owner__", res);
}

bool Universe::createResource(const std::string &uri) {
	owned_.insert(uri);
	subscribers_[uri];
	return true;
}

// TODO (nick) Add URI version and correctly parse URI query parameters
int Universe::numberOfSubscribers(const std::string &res) const {
	auto s = subscribers_.find(res);
	if (s != subscribers_.end()) {
		return s->second.size();
	} else {
		return -1;
	}
}

bool Universe::hasSubscribers(const std::string &res) const {
	// FIXME (nick) Need to parse URI and correct query order
	return numberOfSubscribers(res) > 0;
}

bool Universe::hasSubscribers(const ftl::URI &res) const {
	return numberOfSubscribers(res.to_string()) > 0;
}

bool Universe::_subscribe(const std::string &res) {
	// Need to find who owns the resource
	optional<UUID> pid = findOwner(res);
	
	if (pid) {
		return call<bool>(*pid, "__subscribe__", ftl::net::this_peer, res);
	} else {
		// No resource found
		LOG(WARNING) << "Subscribe to unknown resource: " << res;
		return false;
	}
}

void Universe::__start(Universe * u) {
	u->_run();
}

void Universe::_run() {
	timeval block;

#ifdef WIN32
	WSAData wsaData;
	//If Win32 then load winsock
	if (WSAStartup(MAKEWORD(1, 1), &wsaData) != 0) {
		LOG(ERROR) << "Could not initiate sockets";
		return;
	}
#endif

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
							p->onConnect([this](Peer &p) {
								peer_ids_[p.id()] = &p;
							});
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

