#include <memory.h>
#include <ftl/net.hpp>

#include "ftl/p2p-rm/blob.hpp"

void ftl::rm::Blob::write(size_t offset, const char *data, size_t size) {
	// Sanity check
	if (offset + size > size_) throw -1;
	
	// If local, write direct to data_, otherwise send over network
	if (socket_ != NULL) {
		// Send over network
		//socket_->send(ftl::rm::MEMORY_WRITE, std::string(data,size));
	} else {
		// Copy locally
		memcpy(data_+offset, data, size);
	}
}

void ftl::rm::Blob::read(size_t offset, char *data, size_t size) {
	// Sanity check
	if (offset + size > size_) throw -1;
	
	// If local, write direct to data_, otherwise send over network
	if (socket_ != NULL) {
		// Send over network
		//socket_->send(ftl::rm::MEMORY_WRITE, std::string(data,size));
	} else {
		// Copy locally
		memcpy(data,data_+offset, size);
	}
}

