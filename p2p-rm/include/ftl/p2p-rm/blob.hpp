#ifndef _FTL_P2P_RM_BLOB_HPP_
#define _FTL_P2P_RM_BLOB_HPP_

#include <mutex>
#include <shared_mutex>
#include <ftl/net.hpp>
#include <string>
#include <vector>

namespace ftl {
namespace rm {

/* NOT TO BE USED DIRECTLY */
struct Blob {
	//Blob();
	//~Blob();

	std::vector<ftl::net::Socket*> sockets_;
	char *data_;
	size_t size_;
	std::string uri_;
	uint32_t blobid_;
	
	void finished();
	//void write(size_t offset, const char *data, size_t size);
	//void read(size_t offset, char *data, size_t size);
	void sync(size_t offset, size_t size);
	
	mutable std::shared_mutex mutex_;
};

}
}

#endif // _FTL_P2P_RM_CACHE_HPP_

