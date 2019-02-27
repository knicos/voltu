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
using ftl::URI;
using ftl::rm::Blob;
using ftl::net::Socket;
using ftl::UUID;
using namespace std::chrono;

Cluster::Cluster(const URI &uri, shared_ptr<Listener> l) : Protocol(uri.getBaseURI()), listener_(l) {
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

Cluster::Cluster(const char *uri, shared_ptr<Listener> l) : Protocol(uri), listener_(l) {
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
	bind("getowner", [this](const UUID &u, int ttl, const std::string &uri) { return getOwner_RPC(u,ttl,uri); });
	//bind("getowner", member(&Cluster::getOwner_RPC));
	
	bind("nop", []() { return true; });
	
	bind(P2P_SYNC, [this](uint32_t msg, Socket &s) {
		LOG(INFO) << "Receive blob sync";
	});
}

void Cluster::addPeer(shared_ptr<Socket> &p, bool incoming) {
	LOG(INFO) << ((incoming) ? "Incoming peer added: " : "Peer added: ") << p->getURI();

	//p.setProtocol(this);

	peers_.push_back(p);
	
	if (!incoming) {
		p->onConnect([this](Socket &s) {
			UUID q;
			int ttl = 10;
			
			for (auto b : blobs_) {
				auto o = std::get<0>(s.call<tuple<UUID,uint32_t>>("getowner", q, ttl, b.first));
				if (o != id() && o != UUID(0)) {
					b.second->owner_ = o;
					LOG(INFO) << "Lost ownership of " << b.first.c_str() << " to " << o.to_string();
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
	if (u.getHost() != root_) { LOG(ERROR) << "Non matching host : " << u.getHost() << " - " << root_ << std::endl; return NULL; }

	auto b = blobs_[u.getBaseURI()];
	std::cout << "Blob Found for " << u.getBaseURI() << " = " << (b != nullptr) << std::endl;

	if (!b) {
		LOG(WARNING) << "Unmapped memory requested: " << uri;
	}

	return b;
}

tuple<UUID,uint32_t> Cluster::getOwner_RPC(const UUID &u, int ttl, const std::string &uri) {
	if (requests_.count(u) > 0) return {UUID(0),0};
	requests_[u] = duration_cast<seconds>(steady_clock::now().time_since_epoch()).count();
	
	if (blobs_.count(uri) > 0) {
		return {blobs_[uri]->owner_,0};
	}
	
	vector<tuple<UUID,uint32_t>> results;
	broadcastCall("getowner", results, u, ttl-1, uri);
	
	// TODO Verify all results are equal or empty
	if (results.size() == 0) return {UUID(0),0};
	return results[0];
}

UUID Cluster::getOwner(const std::string &uri) {
	UUID u;
	int ttl = 10;

	return std::get<0>(getOwner_RPC(u, ttl, uri));
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

	UUID o = getOwner(uri);
	if (o == id() || o == UUID(0)) {
		// I am the owner!
		//b->owner_ = "me";
	} else {
		b->owner_ = o;
	}
	
	LOG(INFO) << "Mapping address to " << uri;
	
	blobs_[u.getBaseURI()] = b;
	
	//std::cout << owners << std::endl;
	
	return b;
}

