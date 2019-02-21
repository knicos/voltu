#include "ftl/p2p-rm.hpp"
#include "ftl/p2p-rm/blob.hpp"
#include "ftl/p2p-rm/protocol.hpp"

#include <ftl/uri.hpp>
#include <ftl/net.hpp>

#include <map>
#include <string>

using std::shared_ptr;
using ftl::rm::Cluster;
using ftl::URI;
using ftl::net::Listener;

shared_ptr<Cluster> ftl::rm::cluster(const char *uri, shared_ptr<Listener> l) {
	URI u(uri);
	if (!u.isValid()) return nullptr;
	if (u.getScheme() != ftl::URI::SCHEME_FTL) return nullptr;
	if (u.getPath().size() > 0) return nullptr;

	shared_ptr<Cluster> c(new Cluster(u, l));
	return c;
}

