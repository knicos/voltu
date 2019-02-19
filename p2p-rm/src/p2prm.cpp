#include "ftl/p2p-rm.hpp"

#include <ftl/uri.hpp>
#include <map>

static std::map<std::string, ftl::rm::Blob*> blobs;

void ftl::rm::reset() {
	for (auto x : blobs) {
		delete x;
	}
	blobs.clear();
}

ftl::rm::Blob *ftl::rm::_lookupBlob(const char *uri) {
	URI u(uri);
	if (!u.isValid()) return NULL;
	if (u.getScheme() != ftl::URI::SCHEME_FTL) return NULL;
	if (u.getPathSegment(0) != "memory") return NULL;
	
	return blobs[u.getBaseURI()];
}

ftl::rm::Blob *ftl::rm::_createBlob(const char *uri, size_t size) {
	URI u(uri);
	if (!u.isValid()) return NULL;
	if (u.getScheme() != ftl::URI::SCHEME_FTL) return NULL;
	if (u.getPathSegment(0) != "memory") return NULL;
	
	if (blobs[u.getBaseURI()] != NULL) return NULL;
	
	ftl::rm::Blob *b = new ftl::rm::Blob;

	char *raw = new char[size+sizeof(ftl::rm::Header)];

	b->raw_ = raw;
	b->header_ = (ftl::rm::Header*)raw;
	b->data_ = raw+sizeof(ftl::rm::Header);
	b->size_ = size;
	b->rawsize = size++sizeof(ftl::rm::Header);
	b->socket_ = NULL;
	blobs[u.getBaseURI()] = b;
	return b;
}

