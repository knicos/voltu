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

	if (l != nullptr) {
		l->onConnection([&](shared_ptr<Socket> &s) {
			addPeer(s, true);		
		});
	}
}

Cluster::Cluster(const char *uri, shared_ptr<Listener> l) : Protocol(uri), listener_(l) {
	URI u(uri);
	if (!u.isValid()) return;
	if (u.getScheme() != ftl::URI::SCHEME_FTL) return;
	if (u.getPath().size() > 0) return;

	root_ = u.getHost();

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
	bind("getowner", member(&Cluster::getOwner));
	
	bind("nop", []() { return true; });
	
	bind(P2P_SYNC, [this](uint32_t msg, Socket &s) {
		LOG(INFO) << "Receive blob sync";
	});
}

void Cluster::addPeer(shared_ptr<Socket> &p, bool incoming) {
	LOG(INFO) << ((incoming) ? "Incoming peer added: " : "Peer added: ") << p->getURI();

	//p.setProtocol(this);

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

shared_ptr<Socket> Cluster::addPeer(const char *url) {
	auto sock = ftl::net::connect(url);
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

tuple<string,uint32_t> Cluster::getOwner_RPC(const UUID &u, int ttl, const std::string &uri) {
	if (requests_.count(u) > 0) return {"",0};
	requests_[u] = duration_cast<seconds>(steady_clock::now().time_since_epoch()).count();
	
	std::cout << "GETOWNER" << std::endl;
	if (blobs_.count(uri) != 0) return {blobs_[uri]->owner_,0};
	
	vector<tuple<string,uint32_t>> results;
	broadcastCall("getowner", results, u, ttl-1, uri);
	
	// TODO Verify all results are equal or empty
	if (results.size() == 0) return {"",0};
	return results[0];
}

std::string Cluster::getOwner(const std::string &uri) {
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

