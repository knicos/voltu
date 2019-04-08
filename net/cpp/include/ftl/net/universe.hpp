#ifndef _FTL_NET_UNIVERSE_HPP_
#define _FTL_NET_UNIVERSE_HPP_

#include <ftl/net/peer.hpp>
#include <ftl/net/listener.hpp>
#include <ftl/net/dispatcher.hpp>
#include <ftl/uuid.hpp>
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
	/**
	 * Constructor with a URI base. The base uri is used as a base to validate
	 * resource identifiers. (it may be removed). This creates a new thread
	 * to monitor network sockets.
	 */
	explicit Universe(const std::string &base);

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
	
	/**
	 * Bind a function to an RPC or service call name. This will implicitely
	 * be called by any peer making the request.
	 */
	template <typename F>
	void bind(const std::string &name, F func);
	
	/**
	 * Send a non-blocking RPC call with no return value to all connected
	 * peers.
	 */
	template <typename... ARGS>
	void broadcast(const std::string &name, ARGS... args);
	
	private:
	void _run();
	int _setDescriptors();
	void _installBindings(Peer *);
	
	static void __start(Universe *u);
	
	private:
	bool active_;
	std::string base_;
	std::thread thread_;
	fd_set sfderror_;
	fd_set sfdread_;
	std::vector<ftl::net::Listener*> listeners_;
	std::vector<ftl::net::Peer*> peers_;
	ftl::UUID id_;
	ftl::net::Dispatcher disp_;
};

};  // namespace net
};  // namespace ftl

#endif  // _FTL_NET_UNIVERSE_HPP_

