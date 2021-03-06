/**
 * @file universe.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#ifndef _FTL_NET_UNIVERSE_HPP_
#define _FTL_NET_UNIVERSE_HPP_

#include <ftl/utility/msgpack.hpp>

#include <ftl/configurable.hpp>
#include <ftl/net/peer.hpp>
#include <ftl/net/listener.hpp>
#include <ftl/net/dispatcher.hpp>
#include <ftl/uuid.hpp>
#include <ftl/threads.hpp>
#include <nlohmann/json_fwd.hpp>
#include <vector>
#include <list>
#include <string>
#include <thread>
#include <map>

namespace ftl {
namespace net {

struct Error {
	int errno;
};

struct ReconnectInfo {
	int tries;
	float delay;
	Peer *peer;
};

struct NetImplDetail;

typedef unsigned int callback_t;

/**
 * Represents a group of network peers and their resources, managing the
 * searching of and sharing of resources across peers. Each universe can
 * listen on multiple ports/interfaces for connecting peers, and can connect
 * to any number of peers. The creation of a Universe object also creates a
 * new thread to manage the networking, therefore it is threadsafe but
 * callbacks will execute in a different thread so must also be threadsafe in
 * their actions.
 */
class Universe : public ftl::Configurable {
	public:
	friend class Peer;

	public:
	Universe();
	/**
	 * Constructor with json config object. The config allows listening and
	 * peer connection to be set up automatically.
	 * 
	 * Should be constructed with ftl::create<Universe>(...) and not directly.
	 */
	explicit Universe(nlohmann::json &config);

	/**
	 * The destructor will terminate the network thread before completing.
	 */
	~Universe();

	void start();
	
	/**
	 * Open a new listening port on a given interfaces.
	 *   eg. "tcp://localhost:9000"
	 * @param addr URI giving protocol, interface and port
	 */
	bool listen(const std::string &addr);

	/**
	 * Essential to call this before destroying anything that registered
	 * callbacks or binds for RPC. It will terminate all connections and
	 * stop any network activity but without deleting the net object.
	 */
	void shutdown();
	
	/**
	 * Create a new peer connection.
	 *   eg. "tcp://10.0.0.2:9000"
	 * Supported protocols include tcp and ws.
	 *
	 * @param addr URI giving protocol, interface and port
	 */
	Peer *connect(const std::string &addr);

	bool isConnected(const ftl::URI &uri);
	bool isConnected(const std::string &s);
	
	size_t numberOfPeers() const { return peers_.size(); }

	/**
	 * Will block until all currently registered connnections have completed.
	 * You should not use this, but rather use onConnect.
	 */
	int waitConnections();
	
	Peer *getPeer(const ftl::UUID &pid) const;
	
	//int numberOfSubscribers(const std::string &res) const;

	//bool hasSubscribers(const std::string &res) const;
	//bool hasSubscribers(const ftl::URI &res) const;
	
	/**
	 * Bind a function to an RPC or service call name. This will implicitely
	 * be called by any peer making the request.
	 */
	template <typename F>
	void bind(const std::string &name, F func);

	void unbind(const std::string &name);

	/**
	 * Check if an RPC name is already bound.
	 */
	inline bool isBound(const std::string &name) const { return disp_.isBound(name); }

	/**
	 * Subscribe a function to a resource. The subscribed function is
	 * triggered whenever that resource is published to. It is akin to
	 * RPC broadcast (no return value) to a subgroup of peers.
	 */
	//template <typename F>
	//[[deprecated("Pub sub no longer to be used")]]
	//bool subscribe(const std::string &res, F func);

	/**
	 * Subscribe a function to a resource. The subscribed function is
	 * triggered whenever that resource is published to. It is akin to
	 * RPC broadcast (no return value) to a subgroup of peers.
	 */
	//template <typename F>
	//[[deprecated("Pub sub no longer to be used")]]
	//bool subscribe(const ftl::URI &res, F func);
	
	/**
	 * Send a non-blocking RPC call with no return value to all connected
	 * peers.
	 */
	template <typename... ARGS>
	void broadcast(const std::string &name, ARGS... args);
	
	template <typename R, typename... ARGS>
	R call(const UUID &pid, const std::string &name, ARGS... args);

	/**
	 * Non-blocking Remote Procedure Call using a callback function.
	 * 
	 * @param pid Peer GUID
	 * @param name RPC Function name.
	 * @param cb Completion callback.
	 * @param args A variable number of arguments for RPC function.
	 * 
	 * @return A call id for use with cancelCall() if needed.
	 */
	template <typename R, typename... ARGS>
	int asyncCall(const UUID &pid, const std::string &name,
			std::function<void(const R&)> cb,
			ARGS... args);
	
	template <typename... ARGS>
	bool send(const UUID &pid, const std::string &name, ARGS... args);

	template <typename... ARGS>
	int try_send(const UUID &pid, const std::string &name, ARGS... args);

	template <typename R, typename... ARGS>
	std::optional<R> findOne(const std::string &name, ARGS... args);

	template <typename R, typename... ARGS>
	std::vector<R> findAll(const std::string &name, ARGS... args);
	
	/**
	 * Send a non-blocking RPC call with no return value to all subscribers
	 * of a resource. There may be no subscribers. Note that query parameter
	 * order in the URI string is not important.
	 */
	//template <typename... ARGS>
	//[[deprecated("Pub sub no longer to be used")]]
	//void publish(const std::string &res, ARGS... args);

	/**
	 * Send a non-blocking RPC call with no return value to all subscribers
	 * of a resource. There may be no subscribers. This overload accepts a
	 * URI object directly to enable more efficient modification of parameters.
	 */
	//template <typename... ARGS>
	//[[deprecated("Pub sub no longer to be used")]]
	//void publish(const ftl::URI &res, ARGS... args);
	
	/**
	 * Register your ownership of a new resource. This must be called before
	 * publishing to this resource and before any peers attempt to subscribe.
	 */
	//[[deprecated("Pub sub no longer to be used")]]
	//bool createResource(const std::string &uri);

	//[[deprecated("Pub sub no longer to be used")]]
	//std::optional<ftl::UUID> findOwner(const std::string &res);

	void setLocalID(const ftl::UUID &u) { this_peer = u; };
	const ftl::UUID &id() const { return this_peer; }

	// --- Event Handlers ------------------------------------------------------

	ftl::net::callback_t onConnect(const std::function<void(ftl::net::Peer*)>&);
	ftl::net::callback_t onDisconnect(const std::function<void(ftl::net::Peer*)>&);
	ftl::net::callback_t onError(const std::function<void(ftl::net::Peer*, const ftl::net::Error &)>&);

	void removeCallback(ftl::net::callback_t cbid);

	size_t getSendBufferSize(ftl::URI::scheme_t s);
	size_t getRecvBufferSize(ftl::URI::scheme_t s);

	static inline Universe *getInstance() { return instance_; }
	
	private:
	void _run();
	SOCKET _setDescriptors();
	void _installBindings();
	void _installBindings(Peer *);
	//bool _subscribe(const std::string &res);
	void _cleanupPeers();
	void _notifyConnect(Peer *);
	void _notifyDisconnect(Peer *);
	void _notifyError(Peer *, const ftl::net::Error &);
	void _periodic();
	
	static void __start(Universe *u);
	
	private:
	bool active_;
	ftl::UUID this_peer;
	mutable SHARED_MUTEX net_mutex_;
	RECURSIVE_MUTEX handler_mutex_;
	
	NetImplDetail *impl_;
	
	std::vector<ftl::net::Listener*> listeners_;
	std::vector<ftl::net::Peer*> peers_;
	std::unordered_map<std::string, ftl::net::Peer*> peer_by_uri_;
	//std::map<std::string, std::vector<ftl::UUID>> subscribers_;
	//std::unordered_set<std::string> owned_;
	std::map<ftl::UUID, ftl::net::Peer*> peer_ids_;
	ftl::UUID id_;
	ftl::net::Dispatcher disp_;
	std::list<ReconnectInfo> reconnects_;
	size_t phase_;
	std::list<ftl::net::Peer*> garbage_;
	ftl::Handle garbage_timer_;

	size_t send_size_;
	size_t recv_size_;
	double periodic_time_;
	int reconnect_attempts_;

	// NOTE: Must always be last member
	std::thread thread_;

	struct ConnHandler {
		callback_t id;
		std::function<void(ftl::net::Peer*)> h;
	};

	struct ErrHandler {
		callback_t id;
		std::function<void(ftl::net::Peer*, const ftl::net::Error &)> h;
	};

	// Handlers
	std::list<ConnHandler> on_connect_;
	std::list<ConnHandler> on_disconnect_;
	std::list<ErrHandler> on_error_;

	static callback_t cbid__;
	static Universe *instance_;

	// std::map<std::string, std::vector<ftl::net::Peer*>> subscriptions_;
};

//------------------------------------------------------------------------------

template <typename F>
void Universe::bind(const std::string &name, F func) {
	UNIQUE_LOCK(net_mutex_,lk);
	disp_.bind(name, func,
		typename ftl::internal::func_kind_info<F>::result_kind(),
		typename ftl::internal::func_kind_info<F>::args_kind(),
		typename ftl::internal::func_kind_info<F>::has_peer());
}

/*template <typename F>
bool Universe::subscribe(const std::string &res, F func) {
	return subscribe(ftl::URI(res), func);
}

template <typename F>
bool Universe::subscribe(const ftl::URI &res, F func) {
	bind(res.to_string(), func);
	return _subscribe(res.to_string());
}*/

template <typename... ARGS>
void Universe::broadcast(const std::string &name, ARGS... args) {
	SHARED_LOCK(net_mutex_,lk);
	for (auto p : peers_) {
		if (!p->waitConnection()) continue;
		p->send(name, args...);
	}
}

template <typename R, typename... ARGS>
std::optional<R> Universe::findOne(const std::string &name, ARGS... args) {
	struct SharedData {
		std::atomic_bool hasreturned = false;
		std::mutex m;
		std::condition_variable cv;
		std::optional<R> result;
	};

	auto sdata = std::make_shared<SharedData>();

	auto handler = [sdata](const std::optional<R> &r) {
		std::unique_lock<std::mutex> lk(sdata->m);
		if (r && !sdata->hasreturned) {
			sdata->hasreturned = true;
			sdata->result = r;
		}
		lk.unlock();
		sdata->cv.notify_one();
	};

	{
		SHARED_LOCK(net_mutex_,lk);
		for (auto p : peers_) {
			if (!p->waitConnection()) continue;
			p->asyncCall<std::optional<R>>(name, handler, args...);
		}
	}
	
	// Block thread until async callback notifies us
	std::unique_lock<std::mutex> llk(sdata->m);
	sdata->cv.wait_for(llk, std::chrono::seconds(1), [sdata] {
		return (bool)sdata->hasreturned;
	});

	return sdata->result;
}

template <typename R, typename... ARGS>
std::vector<R> Universe::findAll(const std::string &name, ARGS... args) {
	struct SharedData {
		std::atomic_int returncount = 0;
		std::atomic_int sentcount = 0;
		std::mutex m;
		std::condition_variable cv;
		std::vector<R> results;
	};

	auto sdata = std::make_shared<SharedData>();

	auto handler = [sdata](const std::vector<R> &r) {
		std::unique_lock<std::mutex> lk(sdata->m);
		++sdata->returncount;
		sdata->results.insert(sdata->results.end(), r.begin(), r.end());
		lk.unlock();
		sdata->cv.notify_one();
	};

	{
		SHARED_LOCK(net_mutex_,lk);
		for (auto p : peers_) {
			if (!p->waitConnection()) continue;
			++sdata->sentcount;
			p->asyncCall<std::vector<R>>(name, handler, args...);
		}
	}
	
	std::unique_lock<std::mutex> llk(sdata->m);
	sdata->cv.wait_for(llk, std::chrono::seconds(1), [sdata]{return sdata->returncount == sdata->sentcount; });
	return sdata->results;
}

template <typename R, typename... ARGS>
R Universe::call(const ftl::UUID &pid, const std::string &name, ARGS... args) {
	Peer *p = getPeer(pid);
	if (p == nullptr || !p->isConnected()) {
		if (p == nullptr) throw FTL_Error("Attempting to call an unknown peer : " << pid.to_string());
		else throw FTL_Error("Attempting to call an disconnected peer : " << pid.to_string());
	}
	return p->call<R>(name, args...);
}

template <typename R, typename... ARGS>
int Universe::asyncCall(const ftl::UUID &pid, const std::string &name, std::function<void(const R&)> cb, ARGS... args) {
	Peer *p = getPeer(pid);
	if (p == nullptr || !p->isConnected()) {
		if (p == nullptr) throw FTL_Error("Attempting to call an unknown peer : " << pid.to_string());
		else throw FTL_Error("Attempting to call an disconnected peer : " << pid.to_string());
	}
	return p->asyncCall(name, cb, args...);
}

template <typename... ARGS>
bool Universe::send(const ftl::UUID &pid, const std::string &name, ARGS... args) {
	Peer *p = getPeer(pid);
	if (p == nullptr) {
		//DLOG(WARNING) << "Attempting to call an unknown peer : " << pid.to_string();
		return false;
	}
#ifdef WIN32
	return p->isConnected() && p->send(name, args...) >= 0;
#else
	return p->isConnected() && p->send(name, args...) > 0;
#endif

}

template <typename... ARGS>
int Universe::try_send(const ftl::UUID &pid, const std::string &name, ARGS... args) {
	Peer *p = getPeer(pid);
	if (p == nullptr) {
		//DLOG(WARNING) << "Attempting to call an unknown peer : " << pid.to_string();
		return false;
	}

	return (p->isConnected()) ? p->try_send(name, args...) : -1;
}

/*template <typename... ARGS>
void Universe::publish(const std::string &res, ARGS... args) {
	ftl::URI uri(res);
	publish(uri, args...);
}

template <typename... ARGS>
void Universe::publish(const ftl::URI &res, ARGS... args) {
	std::unique_lock<std::shared_mutex> lk(net_mutex_);
	auto subs = subscribers_[res.getBaseURI()];
	lk.unlock();
	for (auto p : subs) {
		auto peer = getPeer(p);
		if (peer) {
			peer->send(res.getBaseURI(), args...);
		}
	}
}*/

};  // namespace net
};  // namespace ftl

#endif  // _FTL_NET_UNIVERSE_HPP_

