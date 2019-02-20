#include "ftl/p2p-rm.hpp"
#include "ftl/p2p-rm/blob.hpp"
#include "ftl/p2p-rm/protocol.hpp"

#include <ftl/uri.hpp>
#include <ftl/net.hpp>

#include <map>
#include <string>

static std::map<std::string, ftl::rm::Blob*> blobs;

void ftl::rm::reset() {
	for (auto x : blobs) {
		delete x.second;
	}
	blobs.clear();
}

struct SyncHeader {
	uint32_t blobid;
	uint32_t offset;
	uint32_t size;
};

void ftl::rm::_sync(const ftl::rm::Blob &blob, size_t offset, size_t size) {
	// Sanity check
	if (offset + size > size_) throw -1;

	// TODO Delay send to collate many write operations?

	if (blob.sockets_.size() > 0) {
		SyncHeader header{blob.blobid_,static_cast<uint32_t>(offset),static_cast<uint32_t>(size)};
	
		for (auto s : blob.sockets_) {
			// Send over network
			s->send2(P2P_SYNC, std::string((const char*)&header,sizeof(header)),
				std::string(&data_[offset],size));
		}
	}
}

ftl::rm::Blob *ftl::rm::_lookup(const char *uri) {
	URI u(uri);
	if (!u.isValid()) return NULL;
	if (u.getScheme() != ftl::URI::SCHEME_FTL) return NULL;
	if (u.getPathSegment(0) != "memory") return NULL;
	
	return blobs[u.getBaseURI()];
}

ftl::rm::Blob *ftl::rm::_create(const char *uri, char *addr, size_t size, size_t count,
		ftl::rm::flags_t flags, const std::string &tname) {
	URI u(uri);
	if (!u.isValid()) return NULL;
	if (u.getScheme() != ftl::URI::SCHEME_FTL) return NULL;
	if (u.getPathSegment(0) != "memory") return NULL;
	
	if (blobs[u.getBaseURI()] != NULL) return NULL;
	
	ftl::rm::Blob *b = new ftl::rm::Blob;

	b->data_ = addr;
	b->size_ = size;
	b->uri_ = std::string(uri);
	blobs[u.getBaseURI()] = b;

	// TODO : Perhaps broadcast this new allocation?
	return b;
}

