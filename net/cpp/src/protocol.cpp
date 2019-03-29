#include <glog/logging.h>
#include <ftl/net/socket.hpp>
#include <ftl/net/protocol.hpp>
#include <functional>
#include <iostream>

using ftl::net::Socket;
using ftl::net::Protocol;

std::map<std::string,Protocol*> Protocol::protocols__;

Protocol *Protocol::find(const std::string &id) {
	if (protocols__.count(id) > 0) return protocols__[id];
	else return NULL;
}

Protocol::Protocol(const std::string &id) : id_(id) {
	protocols__[id] = this;
}

Protocol::~Protocol() {
	protocols__.erase(id_);
	// TODO Make sure all dependent sockets are closed!
}

void Protocol::bind(int service, std::function<void(uint32_t,Socket&)> func) {
	if (handlers_.count(service) == 0) {
		handlers_[service] = func;
	} else {
		LOG(ERROR) << "Message service " << service << " already bound";
	}
}

void Protocol::dispatchRPC(Socket &s, const std::string &d) {
	disp_.dispatch(s,d);
}

void Protocol::dispatchRaw(uint32_t service, Socket &s) {
	// Lookup raw message handler
	if (handlers_.count(service) > 0) {
		handlers_[service](service, s);
	} else {
		LOG(ERROR) << "Unrecognised service request (" << service << ") from " << s.getURI();
	}
}

