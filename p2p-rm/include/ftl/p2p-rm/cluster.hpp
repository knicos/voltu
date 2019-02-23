#ifndef _FTL_P2P_RM_CLUSTER_HPP_
#define _FTL_P2P_RM_CLUSTER_HPP_

#include "ftl/p2p-rm/mapped_ptr.hpp"
#include "ftl/p2p-rm/internal.hpp"
#include <ftl/p2p-rm/protocol.hpp>

#include <ftl/uri.hpp>
#include <ftl/net/socket.hpp>

#include <type_traits>
#include <memory>
#include <vector>
#include <map>
#include <tuple>
#include <msgpack.hpp>

namespace ftl {
namespace net {
	class Socket;
	class Listener;
};

namespace rm {

class Blob;

class Cluster {
	public:
	Cluster(const ftl::URI &uri, std::shared_ptr<ftl::net::Listener> l);
	~Cluster();
	
	void reset();
	inline void destroy() { reset(); }
	
	/**
	 * Obtain a remote pointer from a URI. A nullptr is returned if the URI is
	 * not valid. If the URI is actually local then a remote pointer is still
	 * returned and may be used normally, although it will possibly result in
	 * unwanted memory copies.
	 */
	template <typename T>
	ftl::mapped_ptr<T> get(const char *uri) {
		auto b = _lookup(uri);
		// TODO Verify type and size
		return ftl::mapped_ptr<T>{b,0};
	}
	
	/**
	 * Get a read-only memory reference from a URI.
	 */
	template <typename T>
	ftl::read_ref<T> getReadable(const char *uri) {
		return get<T>(uri).readable();
	}
	
	/**
	 * Get a read/writable memory reference from a URI.
	 */
	template <typename T>
	ftl::write_ref<T> getWritable(const char *uri) {
		return get<T>(uri).writable();
	}
	
	/**
	 * Register a memory area locally mapped to a given URI. The URI
	 * must not already exist within the peer group.
	 */
	template <typename T>
	ftl::mapped_ptr<T> map(const char *uri, T *addr, size_t size=1) {
		if (std::is_pointer<T>::value) return ftl::null_ptr<T>;
		if (std::is_function<T>::value) return ftl::null_ptr<T>;
		if (std::is_void<T>::value) return ftl::null_ptr<T>;

		if (addr == NULL) return ftl::null_ptr<T>;

		return ftl::mapped_ptr<T>{_create(uri, (char*)addr, sizeof(T), size,
				static_cast<flags_t>(std::is_integral<T>::value * ftl::rm::FLAG_INTEGER |
				std::is_signed<T>::value * ftl::rm::FLAG_SIGNED |
				std::is_trivial<T>::value * ftl::rm::FLAG_TRIVIAL),
				typeid(T).name()),0};
	}
	
	void unmap(const char *uri);

	template <typename T>
	void unmap(ftl::mapped_ptr<T> ptr) {}
	
	/**
	 * Obtain a list or URI memory blocks in the current peer group that match
	 * the provided base URI.
	 */
	std::vector<std::string> search(const char *partial_uri);
	
	/**
	 * Connect to a new peer node using the specified socket.
	 */
	void addPeer(std::shared_ptr<ftl::net::Socket> &s, bool incoming=false);

	/**
	 * Connect to a new peer using a URL string.
	 */
	void addPeer(const char *url);
	
	/**
	 * Allow member functions to be used for RPC calls by binding with 'this'.
	 */
	template <typename R, typename C, typename ...Args>
	auto bind(R(C::*f)(Args...)) {
	  return [this,f](Args... args) -> R { return (this->*f)(std::forward<Args>(args)...); };
	}
	
	std::string getOwner(const std::string &uri);
	
	/**
	 * Make an RPC call to all connected peers and put into a results vector.
	 * This function blocks until all peers have responded or an error /
	 * timeout occurs. The return value indicates the number of failed peers, or
	 * is 0 if all returned.
	 */
	template <typename T, typename... ARGS>
	int broadcastCall(const std::string &name, std::vector<T> &results,
			ARGS... args) {
		int count = 0;
		auto f = [&count,&results](msgpack::object &r) {
			count--;
			results.push_back(r.as<T>());
		};

		for (auto p : peers_) {
			count++;
			p->async_call(name, f, std::forward<ARGS>(args)...);
		}
		
		// TODO Limit in case of no return.
		while (count > 0) {
			ftl::net::wait();
		}
		return count;
	}
	
	private:
	std::string root_;
	std::shared_ptr<ftl::net::Listener> listener_;
	std::vector<std::shared_ptr<ftl::net::Socket>> peers_;
	std::map<std::string, ftl::rm::Blob*> blobs_;
	std::map<int,std::vector<std::tuple<std::shared_ptr<ftl::net::Socket>,std::string>>> rpc_results_;

	ftl::rm::Blob *_lookup(const char *uri);
	Blob *_create(const char *uri, char *addr, size_t size, size_t count,
		ftl::rm::flags_t flags, const std::string &tname);
	void _registerRPC(ftl::net::Socket &s);
};

};
};

#endif // _FTL_P2P_RM_CLUSTER_HPP_

