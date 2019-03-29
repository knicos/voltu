#ifndef _FTL_NET_P2P_HPP_
#define _FTL_NET_P2P_HPP_

#include <ftl/uuid.hpp>
#include <optional>
#include <string>
#include <map>
#include <chrono>
#include <vector>
#include <memory>
#include <ftl/net/protocol.hpp>
#include <ftl/net/socket.hpp>

namespace ftl {
namespace net {

/**
 * Provides the base for p2p calls such as "find a single result" or "find all
 * results" across the peer network. It wraps the underlying rpc mechanism,
 * allowing a p2p rpc broadcast search strategy. It also enables calls to
 * specific peers by peer id and manages the process of finding or initiating
 * the required network connections.
 */
class P2P : public ftl::net::Protocol {
	public:
	P2P(const char *uri);
	P2P(const std::string &uri);
	
	void addPeer(std::shared_ptr<ftl::net::Socket> s) { peers_.push_back(s); };
	
	const UUID &id() const { return id_; }
	
	/**
	 * Bind a member function as an rpc "find one" across peers function.
	 * The function bound is the individual local case only, returning an
	 * optional value. The first peer to return an optional value with an
	 * actual value will be the one used.
	 */
	template <typename R, typename C, typename... Args>
	void bind_find_one(const std::string &name,
			std::optional<R>(C::*f)(Args...));
	
	/**
	 * Bind a member function as an rpc "find all" across peers function.
	 */
	template <typename R, typename C, typename... Args>
	void bind_find_all(const std::string &name,
			std::optional<R>(C::*f)(Args...));
	
	/**
	 * Call an rpc function on all peers (recursively if needed), until one
	 * provides a result.
	 */
	template <typename R, typename... Args>
	std::optional<R> find_one(const std::string &name, Args... args);
	
	/**
	 * Call an rpc function on all peers (recursively), collating all
	 * results into a vector.
	 */
	template <typename R, typename... Args>
	std::vector<R> find_all(const std::string &name, Args... args);
	
	/**
	 * Call an rpc function on a specific peer.
	 */
	template <typename R, typename... Args>
	R call_peer(const ftl::UUID &peer, const std::string &name, Args... args);
	
	/**
	 * Send a raw message to a specific peer. It will attempt to make a direct
	 * connection to that peer if one does not exist (and if the data packet
	 * is sufficiently large, and there are enough connection slots).
	 */
	template <typename... Args>
	void send_peer(const ftl::UUID &peer, uint32_t service, Args... args);
	
	std::vector<std::string> getAddresses(const ftl::UUID &peer);
	std::optional<long int> ping(const ftl::UUID &peer);
	
	private:
	std::unordered_map<ftl::UUID,long int> requests_;
	std::vector<std::shared_ptr<ftl::net::Socket>> peers_;
	
	private:
	template <typename R, typename... Args>
	std::optional<R> _find_one(const std::string &name, const ftl::UUID &u,
			const int &ttl, Args... args);
			
	template <typename R, typename... Args>
	std::vector<R> _find_all(const std::string &name, const ftl::UUID &u,
			const int &ttl, Args... args);
			
	std::optional<long int> _ping(const ftl::UUID &peer, long int time);
	
	void _registerRPC();
	
	private:
	ftl::UUID id_;
};

}; // namespace net
}; // namespace ftl

// --- Template implementations ------------------------------------------------

template <typename R, typename C, typename... Args>
void ftl::net::P2P::bind_find_one(const std::string &name, std::optional<R>(C::*f)(Args...)) {
	bind(name, [this,name,f](const ftl::UUID &u, int ttl, Args... args) -> std::optional<R> {
		if (requests_.count(u) > 0) return {};
		requests_[u] = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
		
		auto result = (static_cast<C*>(this)->*f)(std::forward<Args>(args)...);
		if (result) return result;
		
		// Otherwise we must search again
		if (ttl == 0) return {};
		
		return _find_one<R>(name, u, ttl-1, args...);
	});
}

template <typename R, typename C, typename... Args>
void ftl::net::P2P::bind_find_all(const std::string &name, std::optional<R>(C::*f)(Args...)) {
	bind(name, [this,name,f](const ftl::UUID &u, int ttl, Args... args) -> std::vector<R> {
		std::vector<R> results;
		
		if (requests_.count(u) > 0) return results;
		requests_[u] = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
		
		auto result = (static_cast<C*>(this)->*f)(std::forward<Args>(args)...);
		if (result) results.push_back(*result);
		
		// Otherwise we must search again
		if (ttl == 0) return results;
		
		auto cres = _find_all<R>(name, u, ttl-1, args...);
		if (cres.size() > 0) {
			results.insert(results.end(), cres.begin(), cres.end());
		}

		return results;
	});
}

template <typename R, typename... Args>
std::optional<R> ftl::net::P2P::find_one(const std::string &name, Args... args) {
	ftl::UUID req;
	int ttl = 10;
	return _find_one<R>(name, req, ttl, args...);
}

template <typename R, typename... Args>
std::optional<R> ftl::net::P2P::_find_one(const std::string &name, const ftl::UUID &u, const int &ttl, Args... args) {
	// TODO Use an async approach
	for (auto p : peers_) {
		auto res = p->call<std::optional<R>>(name, u, ttl, args...);
		if (res) return res;
	}
	return {};
}

template <typename R, typename... Args>
std::vector<R> ftl::net::P2P::find_all(const std::string &name, Args... args) {
	ftl::UUID req;
	int ttl = 10;
	return _find_all<R>(name, req, ttl, std::forward<Args...>(args...));
}

template <typename R, typename... Args>
std::vector<R> ftl::net::P2P::_find_all(const std::string &name, const ftl::UUID &u, const int &ttl, Args... args) {
	// TODO Use an async approach
	std::vector<R> results;
	for (auto p : peers_) {
		auto res = p->call<std::vector<R>>(name, u, ttl, args...);
		std::cout << "Result size = " << res.size() << std::endl;
		if (res.size() > 0)
			results.insert(results.end(), res.begin(), res.end());
	}
	return results;
}

#endif // _FTL_NET_P2P_HPP_

