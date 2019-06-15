#include <ftl/net/universe.hpp>
#include <chrono>

#ifdef WIN32
#include <Ws2tcpip.h>
#pragma comment(lib, "Rpcrt4.lib")
#endif

#ifndef WIN32
#include <signal.h>
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
using std::unique_lock;
using std::shared_lock;
using std::mutex;
using std::shared_mutex;
using ftl::config::json_t;
using ftl::net::callback_t;

callback_t ftl::net::Universe::cbid__ = 0;

Universe::Universe() : Configurable(), active_(true), this_peer(ftl::net::this_peer), thread_(Universe::__start, this) {
	_installBindings();
}

Universe::Universe(nlohmann::json &config) :
		Configurable(config), active_(true), this_peer(ftl::net::this_peer), thread_(Universe::__start, this) {

	_installBindings();
}

Universe::~Universe() {
	shutdown();
}

void Universe::start() {
	auto l = get<json_t>("listen");

	if (l && (*l).is_array()) {
		for (auto &ll : *l) {
			listen(ll);
		}
	} else if (l && (*l).is_string()) {
		listen((*l).get<string>());
	}
	
	auto p = get<json_t>("peers");
	if (p && (*p).is_array()) {
		for (auto &pp : *p) {
			connect(pp);
		}
	}
}

void Universe::shutdown() {
	if (!active_) return;
	LOG(INFO) << "Cleanup Network ...";

	active_ = false;
	thread_.join();
	
	for (auto s : peers_) {
		s->rawClose();
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
	UNIQUE_LOCK(net_mutex_,lk);
	listeners_.push_back(l);
	return l->isListening();
}

Peer *Universe::connect(const string &addr) {
	auto p = new Peer(addr.c_str(), this, &disp_);
	if (!p) return nullptr;
	
	if (p->status() != Peer::kInvalid) {
		UNIQUE_LOCK(net_mutex_,lk);
		peers_.push_back(p);
	}
	
	_installBindings(p);
	return p;
}

void Universe::unbind(const std::string &name) {
	UNIQUE_LOCK(net_mutex_,lk);
	disp_.unbind(name);
}

int Universe::waitConnections() {
	int count = 0;
	for (auto p : peers_) {
		if (p->waitConnection()) count++;
	}
	return count;
}

int Universe::_setDescriptors() {
	//Reset all file descriptors
	FD_ZERO(&sfdread_);
	FD_ZERO(&sfderror_);

	SOCKET n = 0;

	// TODO Shared lock for some of the time...
	UNIQUE_LOCK(net_mutex_,lk);

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

			//if (s->isWaiting()) {
				FD_SET(s->_socket(), &sfdread_);
			//}
			FD_SET(s->_socket(), &sfderror_);
		}
	}
	_cleanupPeers();

	return n;
}

void Universe::_installBindings(Peer *p) {
	
}

void Universe::_installBindings() {
	/*bind("__subscribe__", [this](const UUID &id, const string &uri) -> bool {
		LOG(INFO) << "Subscription to " << uri << " by " << id.to_string();
		unique_lock<shared_mutex> lk(net_mutex_);
		subscribers_[ftl::URI(uri).to_string()].push_back(id);
		return true;
	});
	
	bind("__owner__", [this](const std::string &res) -> optional<UUID> {
		if (owned_.count(res) > 0) return this_peer;
		else return {};
	});*/
}

// Note: should be called inside a net lock
void Universe::_cleanupPeers() {

	auto i = peers_.begin();
	while (i != peers_.end()) {
		if (!(*i)->isValid()) {
			Peer *p = *i;
			LOG(INFO) << "Removing disconnected peer: " << p->id().to_string();
			//_notifyDisconnect(p);

			auto ix = peer_ids_.find(p->id());
			if (ix != peer_ids_.end()) peer_ids_.erase(ix);

			i = peers_.erase(i);

			if (p->status() == ftl::net::Peer::kReconnecting) {
				reconnects_.push_back({50, 1.0f, p});
			} else {
				delete p;
			}
		} else {
			i++;
		}
	}
}

Peer *Universe::getPeer(const UUID &id) const {
	auto ix = peer_ids_.find(id);
	if (ix == peer_ids_.end()) return nullptr;
	else return ix->second;
}

/*optional<UUID> Universe::findOwner(const string &res) {
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
		return (int)s->second.size();
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
		return call<bool>(*pid, "__subscribe__", this_peer, res);
	} else {
		// No resource found
		LOG(WARNING) << "Subscribe to unknown resource: " << res;
		return false;
	}
}*/

void Universe::_periodic() {
	auto i = reconnects_.begin();
	while (i != reconnects_.end()) {
		if ((*i).peer->reconnect()) {
			UNIQUE_LOCK(net_mutex_,lk);
			peers_.push_back((*i).peer);
			i = reconnects_.erase(i);
		} else if ((*i).tries > 0) {
			(*i).tries--;
			i++;
		} else {
			delete (*i).peer;
			i = reconnects_.erase(i);
			LOG(WARNING) << "Reconnection to peer failed";
		}
	}
}

void Universe::__start(Universe * u) {
#ifndef WIN32
	signal(SIGPIPE,SIG_IGN);
#endif  // WIN32
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

	auto start = std::chrono::high_resolution_clock::now();

	while (active_) {
		int n = _setDescriptors();
		int selres = 1;

		// Do periodics
		auto now = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsed = now - start;
		if (elapsed.count() >= 1.0) {
			start = now;
			_periodic();
		}

		// It is an error to use "select" with no sockets ... so just sleep
		if (n == 0) {
			std::this_thread::sleep_for(std::chrono::milliseconds(300));
			continue;
		}

		//Wait for a network event or timeout in 3 seconds
		block.tv_sec = 0;
		block.tv_usec = 100000;
		selres = select(n+1, &sfdread_, 0, &sfderror_, &block);

		// NOTE Nick: Is it possible that not all the recvs have been called before I
		// again reach a select call!? What are the consequences of this? A double recv attempt?

		//Some kind of error occured, it is usually possible to recover from this.
		if (selres < 0) {
			switch (errno) {
			case 9	: continue;  // Bad file descriptor = socket closed
			case 4	: continue;  // Interrupted system call ... no problem
			default	: LOG(WARNING) << "Unhandled select error: " << strerror(errno) << "(" << errno << ")";
			}
			continue;
		} else if (selres == 0) {
			// Timeout, nothing to do...
			continue;
		}

		// CHECK Could this mutex be the problem!?
		{
			UNIQUE_LOCK(net_mutex_,lk);

			//If connection request is waiting
			for (auto l : listeners_) {
				if (l && l->isListening()) {
					if (FD_ISSET(l->_socket(), &sfdread_)) {
						int rsize = sizeof(sockaddr_storage);
						sockaddr_storage addr;

						//Finally accept this client connection.
						SOCKET csock = accept(l->_socket(), (sockaddr*)&addr, (socklen_t*)&rsize);

						if (csock != INVALID_SOCKET) {
							auto p = new Peer(csock, this, &disp_);
							peers_.push_back(p);
							_installBindings(p);
						}
					}
				}
			}
		}

		// TODO(Nick) Might switch to shared lock here?
		{
			SHARED_LOCK(net_mutex_, lk);

			// Also check each clients socket to see if any messages or errors are waiting
			for (auto s : peers_) {
				if (s != NULL && s->isValid()) {
					// Note: It is possible that the socket becomes invalid after check but before
					// looking at the FD sets, therefore cache the original socket
					SOCKET sock = s->_socket();
					if (sock == INVALID_SOCKET) continue;

					if (FD_ISSET(sock, &sfderror_)) {
						s->socketError();
						s->close();
						continue;  // No point in reading data...
					}
					//If message received from this client then deal with it
					if (FD_ISSET(sock, &sfdread_)) {
						s->data();
					}
				}
			}
		}

		// TODO(Nick) Don't always need to call this
		//_cleanupPeers();
	}
}

callback_t Universe::onConnect(const std::function<void(ftl::net::Peer*)> &cb) {
	UNIQUE_LOCK(handler_mutex_,lk);
	callback_t id = cbid__++;
	on_connect_.push_back({id, cb});
	return id;
}

callback_t Universe::onDisconnect(const std::function<void(ftl::net::Peer*)> &cb) {
	UNIQUE_LOCK(handler_mutex_,lk);
	callback_t id = cbid__++;
	on_disconnect_.push_back({id, cb});
	return id;
}

callback_t Universe::onError(const std::function<void(ftl::net::Peer*, const ftl::net::Error &)> &cb) {
	UNIQUE_LOCK(handler_mutex_,lk);
	callback_t id = cbid__++;
	on_error_.push_back({id, cb});
	return id;
}

void Universe::removeCallback(callback_t cbid) {
	UNIQUE_LOCK(handler_mutex_,lk);
	{
		auto i = on_connect_.begin();
		while (i != on_connect_.end()) {
			if ((*i).id == cbid) {
				i = on_connect_.erase(i);
			} else {
				i++;
			}
		}
	}

	{
		auto i = on_disconnect_.begin();
		while (i != on_disconnect_.end()) {
			if ((*i).id == cbid) {
				i = on_disconnect_.erase(i);
			} else {
				i++;
			}
		}
	}

	{
		auto i = on_error_.begin();
		while (i != on_error_.end()) {
			if ((*i).id == cbid) {
				i = on_error_.erase(i);
			} else {
				i++;
			}
		}
	}
}

void Universe::_notifyConnect(Peer *p) {
	SHARED_LOCK(handler_mutex_,lk);
	peer_ids_[p->id()] = p;

	for (auto &i : on_connect_) {
		try {
			i.h(p);
		} catch(...) {
			LOG(ERROR) << "Exception inside OnConnect hander: " << i.id;
		}
	}
}

void Universe::_notifyDisconnect(Peer *p) {
	// In all cases, should already be locked outside this function call
	//unique_lock<mutex> lk(net_mutex_);
	SHARED_LOCK(handler_mutex_,lk);
	for (auto &i : on_disconnect_) {
		try {
			i.h(p);
		} catch(...) {
			LOG(ERROR) << "Exception inside OnDisconnect hander: " << i.id;
		}
	}
}

void Universe::_notifyError(Peer *p, const ftl::net::Error &e) {
	// TODO(Nick)
}
