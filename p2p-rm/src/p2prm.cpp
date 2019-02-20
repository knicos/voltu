#include "ftl/p2p-rm.hpp"

#include <ftl/uri.hpp>
#include <map>
#include <string>

static std::map<std::string, ftl::rm::Blob*> blobs;

void ftl::rm::reset() {
	for (auto x : blobs) {
		delete x.second;
	}
	blobs.clear();
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

