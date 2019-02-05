#ifndef _FTL_P2P_RA_HPP_
#define _FTL_P2P_RA_HPP_

#include "ftl/p2p-rm/remote_ptr.hpp"

namespace ftl {
namespace rm {

	void reset();
	ftl::rm::Blob *_lookupBlob(const char *uri);
	ftl::rm::Blob *_createBlob(const char *uri, size_t size);
	
	template <typename T>
	ftl::remote_ptr<T> getPointer(const char *uri) {
		auto b = _lookupBlob(uri);
		// TODO Verify type and size
		return ftl::remote_ptr<T>{b,0};
	}
	
	template <typename T>
	ftl::read_ref<T> getReadable(const char *uri) {
		return getPointer<T>(uri).readable();
	}
	
	template <typename T>
	ftl::write_ref<T> getWritable(const char *uri) {
		return getPointer<T>(uri).writable();
	}
	
	//template <typename T>
	//ftl::rw_ref<T> getReadWrite(const char *uri);
	
	/**
	 * Get a remote write only reference filled with a provided empty value
	 * rather than first performing a remote read to populate.
	 */
	//template <typename T>
	//ftl::write_ref<T> getWritable(const char *uri, T nullvalue);
	
	/**
	 * Make a new memory allocation locally mapped to a given URI. The URI
	 * must not already exist within the peer group, otherwise a nullptr is
	 * returned.
	 */
	template <typename T>
	ftl::remote_ptr<T> alloc(const char *uri, size_t size) {
		auto b = _createBlob(uri, size*sizeof(T));
		return ftl::remote_ptr<T>{b,0};
	}
	
	void free(const char *uri);
	
	template <typename T>
	void free(ftl::remote_ptr<T> p) {
		
	}
	
	/**
	 * Obtain a list or URI memory blocks in the current peer group that match
	 * the provided base URI.
	 */
	std::vector<std::string> list(const char *partial_uri);
	
	void addPeer(ftl::net::raw::Socket *s);

}
}

#endif // _FTL_P2P_RA_HPP_

