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

int Cluster::rpcid_ = 0;

Cluster::Cluster(const URI &uri, shared_ptr<Listener> l) : listener_(l) {
	//auto me = this;
	root_ = uri.getHost();

	if (l != nullptr) {
		l->onConnection([&](shared_ptr<Socket> &s) {
			addPeer(s);		
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

void Cluster::addPeer(shared_ptr<Socket> &p) {
	LOG(INFO) << "Peer added: " << p->getURI();
	//auto me = this;
	peers_.push_back(p);
	/*p->onMessage([&](int service, const std::string &data) {
		std::cout << "MSG " << service << std::endl;
	});*/
	p->bind("getowner", [](const std::string &uri) -> std::string {
		std::cout << "GETOWNER" << std::endl;
		return "";
	});
	
	// TODO Check ownership of my blobs.
	for (auto b : blobs_) {
		getOwner(b.first.c_str());
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
		// Whoops, need to map it first!
	}

	return b;
}

std::string Cluster::getOwner(const char *uri) {
	std::string result;
	int count = 0;

	auto f = [&](msgpack::object &r) {
		count--;
		std::string res = r.as<std::string>();
		if (res.size() > 0) result = res;
	};

	for (auto p : peers_) {
		count++;
		LOG(INFO) << "Request owner of " << uri << " from " << p->getURI();
		p->async_call("getowner", f, std::string(uri));
	}
	
	// TODO Limit in case of no return.
	while (count > 0) {
		ftl::net::wait();
	}
	
	return result;
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

	std::string o = getOwner(uri);
	if (o.size() == 0) {
		// I am the owner!
		std::cout << "I own " << uri << std::endl;
	} else {
		std::cout << "I do not own " << uri << std::endl;
	}
	
	//std::cout << owners << std::endl;
	
	return b;
}

