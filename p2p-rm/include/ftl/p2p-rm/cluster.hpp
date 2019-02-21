#ifndef _FTL_P2P_RM_CLUSTER_HPP_
#define _FTL_P2P_RM_CLUSTER_HPP_

#include "ftl/p2p-rm/mapped_ptr.hpp"
#include "ftl/p2p-rm/internal.hpp"

#include <type_traits>
#include <memory>
#include <vector>

namespace ftl {
namespace net {
	class Socket;
	class Listener;
};

namespace rm {

class Cluster {
	public:
	Cluster(const char *uri, std::shared_ptr<ftl::net::Listener> l);
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

		return ftl::mapped_ptr<T>{_create(this, uri, (char*)addr, sizeof(T), size,
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
	void addPeer(std::shared_ptr<ftl::net::Socket> s);

	/**
	 * Connect to a new peer using a URL string.
	 */
	void addPeer(const char *url);
	
	private:
	std::string uri_;
	std::string root_;
	std::shared_ptr<ftl::net::Listener> listener_;
	std::vector<std::shared_ptr<ftl::net::Socket>> peers_;
};

};
};

#endif // _FTL_P2P_RM_CLUSTER_HPP_

