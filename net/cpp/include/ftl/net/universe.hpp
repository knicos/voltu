#ifndef _FTL_NET_UNIVERSE_HPP_
#define _FTL_NET_UNIVERSE_HPP_

#ifdef _MSC_VER
#include <msgpack_optional.hpp>
#endif

#include <ftl/net/peer.hpp>
#include <ftl/net/listener.hpp>
#include <ftl/net/dispatcher.hpp>
#include <ftl/uuid.hpp>
#include <nlohmann/json.hpp>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <map>

namespace ftl {
namespace net {

/**
 * Represents a group of network peers and their resources, managing the
 * searching of and sharing of resources across peers. Each universe can
 * listen on multiple ports/interfaces for connecting peers, and can connect
 * to any number of peers. The creation of a Universe object also creates a
 * new thread to manage the networking, therefore it is threadsafe but
 * callbacks will execute in a different thread so must also be threadsafe in
 * their actions.
 */
class Universe {
	public:
	Universe();
	/**
	 * Constructor with json config object. The config allows listening and
	 * peer connection to be set up automatically.
	 */
	explicit Universe(nlohmann::json &config);

	/**
	 * The destructor will terminate the network thread before completing.
	 */
	~Universe();
	
	/**
	 * Open a new listening port on a given interfaces.
	 *   eg. "tcp://localhost:9000"
	 * @param addr URI giving protocol, interface and port
	 */
	bool listen(const std::string &addr);
	
	/**
	 * Create a new peer connection.
	 *   eg. "tcp://10.0.0.2:9000"
	 * Supported protocols include tcp and ws.
	 *
	 * @param addr URI giving protocol, interface and port
	 */
	bool connect(const std::string &addr);
	
	int numberOfPeers() const { return peers_.size(); }
	
	Peer *getPeer(const ftl::UUID &pid) const;
	
	int numberOfSubscribers(const std::string &res) const;

	bool hasSubscribers(const std::string &res) const;
	bool hasSubscribers(const ftl::URI &res) const;
	
	/**
	 * Bind a function to an RPC or service call name. This will implicitely
	 * be called by any peer making the request.
	 */
	template <typename F>
	void bind(const std::string &name, F func);
	
	/**
	 * Subscribe a function to a resource. The subscribed function is
	 * triggered whenever that resource is published to. It is akin to
	 * RPC broadcast (no return value) to a subgroup of peers.
	 */
	template <typename F>
	bool subscribe(const std::string &res, F func);

	/**
	 * Subscribe a function to a resource. The subscribed function is
	 * triggered whenever that resource is published to. It is akin to
	 * RPC broadcast (no return value) to a subgroup of peers.
	 */
	template <typename F>
	bool subscribe(const ftl::URI &res, F func);
	
	/**
	 * Send a non-blocking RPC call with no return value to all connected
	 * peers.
	 */
	template <typename... ARGS>
	void broadcast(const std::string &name, ARGS... args);
	
	template <typename R, typename... ARGS>
	R call(const UUID &pid, const std::string &name, ARGS... args);
	
	template <typename R, typename... ARGS>
	std::optional<R> findOne(const std::string &name, ARGS... args);
	
	/**
	 * Send a non-blocking RPC call with no return value to all subscribers
	 * of a resource. There may be no subscribers. Note that query parameter
	 * order in the URI string is not important.
	 */
	template <typename... ARGS>
	void publish(const std::string &res, ARGS... args);

	/**
	 * Send a non-blocking RPC call with no return value to all subscribers
	 * of a resource. There may be no subscribers. This overload accepts a
	 * URI object directly to enable more efficient modification of parameters.
	 */
	template <typename... ARGS>
	void publish(const ftl::URI &res, ARGS... args);
	
	/**
	 * Register your ownership of a new resource. This must be called before
	 * publishing to this resource and before any peers attempt to subscribe.
	 */
	bool createResource(const std::string &uri);

	std::optional<ftl::UUID> findOwner(const std::string &res);
	
	private:
	void _run();
	int _setDescriptors();
	void _installBindings();
	void _installBindings(Peer *);
	bool _subscribe(const std::string &res);
	
	static void __start(Universe *u);
	
	private:
	bool active_;
	nlohmann::json config_;
	std::mutex net_mutex_;
	fd_set sfderror_;
	fd_set sfdread_;
	std::vector<ftl::net::Listener*> listeners_;
	std::vector<ftl::net::Peer*> peers_;
	std::map<std::string, std::vector<ftl::UUID>> subscribers_;
	std::unordered_set<std::string> owned_;
	std::map<ftl::UUID, ftl::net::Peer*> peer_ids_;
	ftl::UUID id_;
	ftl::net::Dispatcher disp_;
	std::thread thread_;
	// std::map<std::string, std::vector<ftl::net::Peer*>> subscriptions_;
};

//------------------------------------------------------------------------------

template <typename F>
void Universe::bind(const std::string &name, F func) {
	disp_.bind(name, func,
		typename ftl::internal::func_kind_info<F>::result_kind(),
	    typename ftl::internal::func_kind_info<F>::args_kind());
}

template <typename F>
bool Universe::subscribe(const std::string &res, F func) {
	return subscribe(ftl::URI(res), func);
}

template <typename F>
bool Universe::subscribe(const ftl::URI &res, F func) {
	bind(res.to_string(), func);
	return _subscribe(res.to_string());
}

template <typename... ARGS>
void Universe::broadcast(const std::string &name, ARGS... args) {
	for (auto p : peers_) {
		p->send(name, args...);
	}
}

template <typename R, typename... ARGS>
std::optional<R> Universe::findOne(const std::string &name, ARGS... args) {
	bool hasreturned = false;
	std::mutex m;
	std::condition_variable cv;
	
	std::optional<R> result;

	auto handler = [&](const std::optional<R> &r) {
		std::unique_lock<std::mutex> lk(m);
		if (hasreturned || !r) return;
		hasreturned = true;
		result = r;
		lk.unlock();
		cv.notify_one();
	};

	std::map<Peer*, int> record;
	for (auto p : peers_) {
		record[p] = p->asyncCall<std::optional<R>>(name, handler, std::forward<ARGS>(args)...);
	}
	
	{  // Block thread until async callback notifies us
		std::unique_lock<std::mutex> lk(m);
		cv.wait_for(lk, std::chrono::seconds(1), [&hasreturned]{return hasreturned;});

		// Cancel any further results
		for (auto p : peers_) {
			auto m = record.find(p);
			if (m != record.end()) {
				p->cancelCall(m->second);
			}
		}
	}

	return result;
}

template <typename R, typename... ARGS>
R Universe::call(const ftl::UUID &pid, const std::string &name, ARGS... args) {
	Peer *p = getPeer(pid);
	if (p == nullptr) {
		LOG(ERROR) << "Attempting to call an unknown peer : " << pid.to_string();
		throw -1;
	}
	return p->call<R>(name, args...);
}

template <typename... ARGS>
void Universe::publish(const std::string &res, ARGS... args) {
	ftl::URI uri(res);
	publish(uri, args...);
}

template <typename... ARGS>
void Universe::publish(const ftl::URI &res, ARGS... args) {
	auto subs = subscribers_[res.getBaseURI()];
	for (auto p : subs) {
		auto peer = getPeer(p);
		if (peer) {
			peer->send(res.getBaseURI(), args...);
		}
	}
}

};  // namespace net
};  // namespace ftl

#endif  // _FTL_NET_UNIVERSE_HPP_

