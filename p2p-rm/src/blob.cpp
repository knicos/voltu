#include <memory.h>

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

