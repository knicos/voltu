#ifndef _FTL_NET_UNIVERSE_HPP_
#define _FTL_NET_UNIVERSE_HPP_

#include <ftl/net/peer.hpp>
#include <ftl/net/listener.hpp>
#include <ftl/net/dispatcher.hpp>
#include <ftl/uuid.hpp>
#include <nlohmann/json.hpp>
#include <vector>
#include <string>
#include <thread>

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
	 * Send a non-blocking RPC call with no return value to all connected
	 * peers.
	 */
	template <typename... ARGS>
	void broadcast(const std::string &name, ARGS... args);
	
	/**
	 * Send a non-blocking RPC call with no return value to all subscribers
	 * of a resource. There may be no subscribers.
	 */
	template <typename... ARGS>
	void publish(const std::string &res, ARGS... args);
	
	// TODO(nick) Add find_one, find_all, call_any ...
	
	private:
	void _run();
	int _setDescriptors();
	void _installBindings(Peer *);
	
	static void __start(Universe *u);
	
	private:
	bool active_;
	nlohmann::json config_;
	std::thread thread_;
	fd_set sfderror_;
	fd_set sfdread_;
	std::vector<ftl::net::Listener*> listeners_;
	std::vector<ftl::net::Peer*> peers_;
	ftl::UUID id_;
	ftl::net::Dispatcher disp_;
	
	// std::map<std::string, std::vector<ftl::net::Peer*>> subscriptions_;
};

//------------------------------------------------------------------------------

template <typename F>
void Universe::bind(const std::string &name, F func) {
	disp_.bind(name, func,
		typename ftl::internal::func_kind_info<F>::result_kind(),
	    typename ftl::internal::func_kind_info<F>::args_kind());
}

template <typename... ARGS>
void Universe::broadcast(const std::string &name, ARGS... args) {
	for (auto p : peers_) {
		p->send(name, args...);
	}
}

};  // namespace net
};  // namespace ftl

#endif  // _FTL_NET_UNIVERSE_HPP_

