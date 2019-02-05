#ifndef _FTL_P2P_RA_BLOB_HPP_
#define _FTL_P2P_RA_BLOB_HPP_

#include <mutex>
#include <shared_mutex>
#include <ftl/net.hpp>

namespace ftl {
namespace rm {

/* NOT TO BE USED DIRECTLY */
struct Blob {
	ftl::net::raw::Socket *socket_;
	char *data_;
	size_t size_;
	
	void finished();
	void write(size_t offset, const char *data, size_t size);
	void read(size_t offset, char *data, size_t size);
	
	mutable std::shared_mutex mutex_;
};

}
}

#endif // _FTL_P2P_RA_CACHE_HPP_

