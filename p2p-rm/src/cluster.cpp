#include "ftl/p2p-rm.hpp"
#include "ftl/p2p-rm/blob.hpp"
#include "ftl/p2p-rm/protocol.hpp"
#include <ftl/p2p-rm/cluster.hpp>

#include <ftl/net.hpp>
#include <ftl/net/listener.hpp>
#include <ftl/net/socket.hpp>

#include <map>
#include <string>
#include <iostream>

using ftl::rm::Cluster;
using ftl::net::Listener;
using std::map;
using std::shared_ptr;
using ftl::URI;
using ftl::rm::Blob;
using ftl::net::Socket;

Cluster::Cluster(const URI &uri, shared_ptr<Listener> l) : listener_(l) {
	auto me = this;
	root_ = uri.getHost();

	if (l != nullptr) {
		l->onConnection([&](shared_ptr<Socket> s) {
			me->addPeer(s);		
		});
	}
}

Cluster::~Cluster() {
	reset();
}

void Cluster::reset() {
	for (auto x : blobs_) {
		delete x.second;
	}
	blobs_.clear();
}

void Cluster::addPeer(shared_ptr<Socket> p) {

}

struct SyncHeader {
	uint32_t blobid;
	uint32_t offset;
	uint32_t size;
};

void ftl::rm::_sync(const Blob &blob, size_t offset, size_t size) {
	// Sanity check
	if (offset + size > blob.size_) throw -1;

	// TODO Delay send to collate many write operations?

	if (blob.sockets_.size() > 0) {
		SyncHeader header{blob.blobid_,static_cast<uint32_t>(offset),static_cast<uint32_t>(size)};
	
		for (auto s : blob.sockets_) {
			// Send over network
			s->send2(P2P_SYNC, std::string((const char*)&header,sizeof(header)),
				std::string(&blob.data_[offset],size));
		}
	}
}

Blob *Cluster::_lookup(const char *uri) {
	URI u(uri);
	if (!u.isValid()) return NULL;
	if (u.getScheme() != ftl::URI::SCHEME_FTL) return NULL;
	//if (u.getPathSegment(0) != "memory") return NULL;
	if (u.getHost() != root_) { std::cerr << "Non matching host : " << u.getHost() << " - " << root_ << std::endl; return NULL; }

	auto b = blobs_[u.getBaseURI()];
	std::cout << "Blob Found for " << u.getBaseURI() << " = " << (b != nullptr) << std::endl;

	if (b == nullptr) {
		// Must do a p2p search for this URI...
		int rpcid = rpcid_++;

		for (auto p : peers_) {
			p->send(P2P_FINDOWNER, ftl::net::rpc_pack(rpcid,uri));
		}

		int limit = 10;
		while (limit >= 0 && !rpc_results_[rpcid] == nullptr) {
			ftl::net::wait();
			limit--;
		}

		if (rpc_results[rpcid]) {
			// Unpack the data
			auto res = rpc_results[rpcid];
			
		} else {
			// No results;
		}
	}

	return b;
}

Blob *Cluster::_create(const char *uri, char *addr, size_t size, size_t count,
		ftl::rm::flags_t flags, const std::string &tname) {
	URI u(uri);
	if (!u.isValid()) return NULL;
	if (u.getScheme() != ftl::URI::SCHEME_FTL) return NULL;
	if (u.getHost() != root_) { std::cerr << "Non matching host : " << u.getHost() << " - " << root_ << std::endl; return NULL; }
	
	if (blobs_[u.getBaseURI()] != NULL) return NULL;
	
	Blob *b = new Blob;

	b->data_ = addr;
	b->size_ = size;
	b->uri_ = std::string(uri);
	blobs_[u.getBaseURI()] = b;

	// TODO : Perhaps broadcast this new allocation?
	return b;
}

