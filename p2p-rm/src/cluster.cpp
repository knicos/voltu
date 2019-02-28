#include <glog/logging.h>
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
#include <chrono>

using ftl::rm::Cluster;
using ftl::net::Listener;
using std::map;
using std::vector;
using std::tuple;
using std::shared_ptr;
using std::string;
using std::optional;
using ftl::URI;
using ftl::rm::Blob;
using ftl::net::Socket;
using ftl::UUID;
using ftl::net::p2p;
using namespace std::chrono;

Cluster::Cluster(const URI &uri, shared_ptr<Listener> l) : p2p(uri.getBaseURI()), listener_(l) {
	//auto me = this;
	root_ = uri.getHost();
	
	_registerRPC();

	if (l != nullptr) {
		l->onConnection([&](shared_ptr<Socket> &s) {
			addPeer(s, true);		
		});
	}
	
	LOG(INFO) << "Cluster UUID = " << id_.to_string();
}

Cluster::Cluster(const char *uri, shared_ptr<Listener> l) : p2p(uri), listener_(l) {
	URI u(uri);
	if (!u.isValid()) return;
	if (u.getScheme() != ftl::URI::SCHEME_FTL) return;
	if (u.getPath().size() > 0) return;

	root_ = u.getHost();
	
	_registerRPC();

	if (l != nullptr) {
		l->setProtocol(this);
		l->onConnection([&](shared_ptr<Socket> &s) {
			addPeer(s, true);		
		});
	}
	
	LOG(INFO) << "Cluster UUID = " << id_.to_string();
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

void Cluster::_registerRPC() {
	bind_find_one("getowner", &Cluster::getOwner);
	
	bind(P2P_SYNC, [this](uint32_t msg, Socket &s) {
		LOG(INFO) << "Receive blob sync";
	});
}

void Cluster::addPeer(shared_ptr<Socket> &p, bool incoming) {
	LOG(INFO) << ((incoming) ? "Incoming peer added: " : "Peer added: ") << p->getURI();

	//p.setProtocol(this);

	//peers_.push_back(p);
	p2p::addPeer(p);
	
	if (!incoming) {
		p->onConnect([this](Socket &s) {			
			for (auto b : blobs_) {
				auto o = find_one<UUID>("getowner", b.first);
				if (o && *o != id()) {
					b.second->owner_ = *o;
					LOG(INFO) << "Lost ownership of " << b.first.c_str() << " to " << (*o).to_string();
				}
			}
		});
	}
}

shared_ptr<Socket> Cluster::addPeer(const char *url) {
	auto sock = ftl::net::connect(url);
	sock->setProtocol(this);
	addPeer(sock);
	return sock;
}

Blob *Cluster::_lookup(const char *uri) {
	URI u(uri);
	if (!u.isValid()) return NULL;
	if (u.getScheme() != ftl::URI::SCHEME_FTL) return NULL;
	//if (u.getPathSegment(0) != "memory") return NULL;
	if (u.getHost() != root_) { LOG(ERROR) << "Non matching URI base : " << u.getHost() << " - " << root_ << std::endl; return NULL; }

	auto b = blobs_[u.getBaseURI()];
	std::cout << "Blob Found for " << u.getBaseURI() << " = " << (b != nullptr) << std::endl;

	if (!b) {
		LOG(WARNING) << "Unmapped memory requested: " << uri;
	}

	return b;
}

optional<UUID> Cluster::getOwner(const std::string &uri) {
	if (blobs_.count(uri) > 0) {
		return blobs_[uri]->owner_;
	}
	return {};
}

Blob *Cluster::_create(const char *uri, char *addr, size_t size, size_t count,
		ftl::rm::flags_t flags, const std::string &tname) {
	URI u(uri);
	if (!u.isValid()) return NULL;
	if (u.getScheme() != ftl::URI::SCHEME_FTL) return NULL;
	if (u.getHost() != root_) { std::cerr << "Non matching host : " << u.getHost() << " - " << root_ << std::endl; return NULL; }
	
	if (blobs_.count(u.getBaseURI()) > 0) {
		LOG(WARNING) << "Mapping already exists for " << uri;
		return blobs_[u.getBaseURI()];
	}
	
	Blob *b = new Blob;

	b->cluster_ = this;
	b->data_ = addr;
	b->size_ = size;
	b->uri_ = std::string(uri);
	b->owner_ = id(); // I am initial owner by default...

	auto o = find_one<UUID>("getowner", uri);
	
	if ((o && *o == id()) || !o) {

	} else {
		b->owner_ = *o;
	}
	
	LOG(INFO) << "Mapping address to " << uri;
	
	blobs_[u.getBaseURI()] = b;
	
	//std::cout << owners << std::endl;
	
	return b;
}

