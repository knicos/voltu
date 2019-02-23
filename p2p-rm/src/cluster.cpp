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
#include <vector>

using ftl::rm::Cluster;
using ftl::net::Listener;
using std::map;
using std::vector;
using std::shared_ptr;
using std::string;
using ftl::URI;
using ftl::rm::Blob;
using ftl::net::Socket;

Cluster::Cluster(const URI &uri, shared_ptr<Listener> l) : listener_(l) {
	//auto me = this;
	root_ = uri.getHost();

	if (l != nullptr) {
		l->onConnection([&](shared_ptr<Socket> &s) {
			addPeer(s, true);		
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

void Cluster::_registerRPC(Socket &s) {
	//s.bind("getowner", [this](const std::string &u) { getOwner(u.c_str()); });
	s.bind("getowner", bind(&Cluster::getOwner));
}

void Cluster::addPeer(shared_ptr<Socket> &p, bool incoming) {
	LOG(INFO) << ((incoming) ? "Incoming peer added: " : "Peer added: ") << p->getURI();

	peers_.push_back(p);
	_registerRPC(*p);
	
	if (!incoming) {
		p->onConnect([this](Socket &s) {
			for (auto b : blobs_) {
				auto o = s.call<string>("getowner", b.first);
				if (o.size() > 0) {
					b.second->owner_ = o;
					LOG(INFO) << "Lost ownership of " << b.first.c_str() << " to " << o;
				}
			}
		});
	}
}

void Cluster::addPeer(const char *url) {
	auto sock = ftl::net::connect(url);
	addPeer(sock);
}

Blob *Cluster::_lookup(const char *uri) {
	URI u(uri);
	if (!u.isValid()) return NULL;
	if (u.getScheme() != ftl::URI::SCHEME_FTL) return NULL;
	//if (u.getPathSegment(0) != "memory") return NULL;
	if (u.getHost() != root_) { LOG(ERROR) << "Non matching host : " << u.getHost() << " - " << root_ << std::endl; return NULL; }

	auto b = blobs_[u.getBaseURI()];
	std::cout << "Blob Found for " << u.getBaseURI() << " = " << (b != nullptr) << std::endl;

	if (!b) {
		LOG(WARNING) << "Unmapped memory requested: " << uri;
	}

	return b;
}

std::string Cluster::getOwner(const std::string &uri) {
	vector<string> results;
	
	std::cout << "GETOWNER" << std::endl;
	if (blobs_.count(uri) != 0) return blobs_[uri]->owner_;

	broadcastCall("getowner", results, uri);
	
	// TODO Verify all results are equal or empty
	if (results.size() == 0) return "";
	return results[0];
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
	b->owner_ = "";
	blobs_[u.getBaseURI()] = b;

	std::string o = getOwner(uri);
	if (o.size() == 0) {
		// I am the owner!
		std::cout << "I own " << uri << std::endl;
		b->owner_ = "me";
	} else {
		std::cout << "I do not own " << uri << std::endl;
		b->owner_ = o;
	}
	
	//std::cout << owners << std::endl;
	
	return b;
}

