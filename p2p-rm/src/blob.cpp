#include <memory.h>
#include <ftl/net.hpp>

#include "ftl/p2p-rm/blob.hpp"

#define MEMORY_SYNC		0x1000

struct Header {
	uint32_t blobid;
	uint32_t offset;
	uint32_t size;
};

void ftl::rm::Blob::sync(size_t offset, size_t size) {
	// Sanity check
	if (offset + size > size_) throw -1;

	// TODO Delay send to collate many write operations?

	if (sockets_.size() > 0) {
		Header header{blobid_,static_cast<uint32_t>(offset),static_cast<uint32_t>(size)};
	
		// If local, write direct to data_, otherwise send over network
		for (auto s : sockets_) {
			// Send over network
			s->send2(MEMORY_SYNC, std::string((const char*)&header,sizeof(header)),
				std::string(&data_[offset],size));
		}
	}
}

/*void ftl::rm::Blob::write(size_t offset, const char *data, size_t size) {
	// Sanity check
	if (offset + size > size_) throw -1;
	
	// If local, write direct to data_, otherwise send over network
	if (socket_ != NULL) {
		Header header{blobid_,static_cast<uint32_t>(offset),static_cast<uint32_t>(size)};

		// Send over network
		socket_->send2(MEMORY_WRITE, std::string((const char*)&header,sizeof(header)),
			std::string(data,size));
	} else {
		// Copy locally
		memcpy(data_+offset, data, size);
	}
}*/

/*void ftl::rm::Blob::read(size_t offset, char *data, size_t size) {
	// Sanity check
	if (offset + size > size_) throw -1;
	
	// If local, write direct to data_, otherwise send over network
	if (socket_ != NULL) {
		
	} else {
		// Copy locally
		memcpy(data,data_+offset, size);
	}
}*/

