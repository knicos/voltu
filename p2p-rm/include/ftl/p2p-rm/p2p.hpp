#ifndef _FTL_RM_P2P_HPP_
#define _FTL_RM_P2P_HPP_

#include <ftl/uuid.hpp>
#include <optional>
#include <string>
#include <map>
#include <chrono>
#include <vector>
#include <memory>
#include <ftl/net/protocol.hpp>
#include <ftl/net/socket.hpp>
#include <iostream>

namespace ftl {
namespace net {

class p2p : public ftl::net::Protocol {
	public:
	p2p(const char *uri) : Protocol(uri) {}
	p2p(const std::string &uri) : Protocol(uri) {}
	
	void addPeer(std::shared_ptr<ftl::net::Socket> s) { peers_.push_back(s); };
	
	template <typename R, typename C, typename... Args>
	void bind_find_one(const std::string &name, std::optional<R>(C::*f)(Args...)) {
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
	void bind_find_all(const std::string &name, std::optional<R>(C::*f)(Args...)) {
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
	std::optional<R> find_one(const std::string &name, Args... args) {
		ftl::UUID req;
		int ttl = 10;
		return _find_one<R>(name, req, ttl, args...);
	}
	
	template <typename R, typename... Args>
	std::optional<R> _find_one(const std::string &name, const ftl::UUID &u, const int &ttl, Args... args) {
		// TODO Use an async approach
		for (auto p : peers_) {
			auto res = p->call<std::optional<R>>(name, u, ttl, args...);
			if (res) return res;
		}
		return {};
	}
	
	template <typename R, typename... Args>
	std::vector<R> find_all(const std::string &name, Args... args) {
		ftl::UUID req;
		int ttl = 10;
		return _find_all<R>(name, req, ttl, std::forward<Args...>(args...));
	}
	
	template <typename R, typename... Args>
	std::vector<R> _find_all(const std::string &name, const ftl::UUID &u, const int &ttl, Args... args) {
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
	
	/*R find_all(const std::string &name, Args... args) {
	
	}*/
	
	private:
	std::unordered_map<ftl::UUID,long int> requests_;
	std::vector<std::shared_ptr<ftl::net::Socket>> peers_;
};

}; // namespace net
}; // namespace ftl

#endif // _FTL_RM_P2P_HPP_

