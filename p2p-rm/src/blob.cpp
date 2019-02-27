#include <glog/logging.h>
#include <ftl/p2p-rm/blob.hpp>
#include <ftl/net/socket.hpp>
#include <ftl/p2p-rm/protocol.hpp>

#include <iostream>

using ftl::net::array;

struct SyncHeader {
	uint32_t blobid;
	uint32_t offset;
	uint32_t size;
};

void ftl::rm::_sync(const Blob &blob, size_t offset, size_t size) {
	// Sanity check
	if (offset + size > blob.size_) {
		LOG(ERROR) << "Memory overrun during sync";
		return;
	}

	// TODO Delay send to collate many write operations?
	
	LOG(INFO) << "Synchronise blob " << blob.blobid_;

	if (blob.sockets_.size() > 0) {
		SyncHeader header{blob.blobid_,static_cast<uint32_t>(offset),static_cast<uint32_t>(size)};
	
		for (auto s : blob.sockets_) {
			// Send over network
			s->send(P2P_SYNC, std::string((const char*)&header,sizeof(header)),
				array{&blob.data_[offset],size});
		}
	}
}

void ftl::rm::Blob::finished() {

}

