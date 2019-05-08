#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <glog/logging.h>
#include <ctpl_stl.h>

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <fcntl.h>
#ifdef WIN32
#include <winsock2.h>
#include <Ws2tcpip.h>
#include <windows.h>
#endif

#ifdef WIN32
#pragma comment(lib, "Ws2_32.lib")
#pragma comment(lib, "Rpcrt4.lib")
#endif

#include <ftl/uri.hpp>
#include <ftl/net/peer.hpp>
#include <ftl/net/ws_internal.hpp>
#include <ftl/config.h>
#include "net_internal.hpp"

#ifndef WIN32
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#endif

#include <iostream>
#include <memory>
#include <algorithm>
#include <tuple>

using std::tuple;
using std::get;
using ftl::net::Peer;
using ftl::URI;
using ftl::net::ws_connect;
using ftl::net::Dispatcher;

/*static std::string hexStr(const std::string &s)
{
	const char *data = s.data();
	int len = s.size();
    std::stringstream ss;
    ss << std::hex;
    for(int i=0;i<len;++i)
        ss << std::setw(2) << std::setfill('0') << (int)data[i];
    return ss.str();
}*/

int Peer::rpcid__ = 0;

// Global peer UUID
ftl::UUID ftl::net::this_peer;

static ctpl::thread_pool pool(5);

// TODO(nick) Move to tcp_internal.cpp
static int tcpConnect(URI &uri) {
	int rc;
	sockaddr_in destAddr;

	#ifdef WIN32
	WSAData wsaData;
	if (WSAStartup(MAKEWORD(1,1), &wsaData) != 0) {
		LOG(ERROR) << "Could not initiate sockets";
		return INVALID_SOCKET;
	}
	#endif
	
	//We want a TCP socket
	int csocket = socket(AF_INET, SOCK_STREAM, 0);

	if (csocket == INVALID_SOCKET) {
		LOG(ERROR) << "Unable to create TCP socket";
		return INVALID_SOCKET;
	}

	#ifdef WIN32
	HOSTENT *host = gethostbyname(uri.getHost().c_str());
	#else
	hostent *host = gethostbyname(uri.getHost().c_str());
	#endif

	if (host == NULL) {
		#ifndef WIN32
		close(csocket);
		#else
		closesocket(csocket);
		#endif

		LOG(ERROR) << "Address not found : " << uri.getHost() << std::endl;
		return INVALID_SOCKET;
	}

	destAddr.sin_family = AF_INET;
	destAddr.sin_addr.s_addr = ((in_addr *)(host->h_addr))->s_addr;
	destAddr.sin_port = htons(uri.getPort());

	// Make nonblocking
	/*long arg = fcntl(csocket, F_GETFL, NULL));
	arg |= O_NONBLOCK;
	fcntl(csocket, F_SETFL, arg) < 0)*/
	
	rc = ::connect(csocket, (struct sockaddr*)&destAddr, sizeof(destAddr));

	if (rc < 0) {
		if (errno == EINPROGRESS) {

		} else {
			#ifndef WIN32
			close(csocket);
			#else
			closesocket(csocket);
			#endif

			LOG(ERROR) << "Could not connect to " << uri.getBaseURI();

			return INVALID_SOCKET;
		}
	}

	// Make blocking again
	/*long arg = fcntl(csocket, F_GETFL, NULL);
	arg &= (~O_NONBLOCK);
	fcntl(csocket, F_SETFL, arg);*/

	return csocket;
}

Peer::Peer(int s, Dispatcher *d) : sock_(s) {
	status_ = (s == INVALID_SOCKET) ? kInvalid : kConnecting;
	_updateURI();
	
	disp_ = new Dispatcher(d);
	
	is_waiting_ = true;
	
	// Send the initiating handshake if valid
	if (status_ == kConnecting) {
		// Install return handshake handler.
		bind("__handshake__", [this](uint64_t magic, uint32_t version, UUID pid) {
			LOG(INFO) << "Handshake 2 received";
			if (magic != ftl::net::kMagic) {
				close();
				LOG(ERROR) << "Invalid magic during handshake";
			} else {
				status_ = kConnected;
				version_ = version;
				peerid_ = pid;
				
				_trigger(open_handlers_);
			}
		});

		send("__handshake__", ftl::net::kMagic, ftl::net::kVersion, ftl::net::this_peer); 
	}
}

Peer::Peer(const char *pUri, Dispatcher *d) : uri_(pUri) {	
	URI uri(pUri);
	
	status_ = kInvalid;
	sock_ = INVALID_SOCKET;
	
	disp_ = new Dispatcher(d);

	scheme_ = uri.getProtocol();
	if (uri.getProtocol() == URI::SCHEME_TCP) {
		sock_ = tcpConnect(uri);
		if (sock_ != INVALID_SOCKET) status_ = kConnecting;
		else status_ = kReconnecting;
	} else if (uri.getProtocol() == URI::SCHEME_WS) {
		LOG(INFO) << "Websocket connect " << uri.getPath();
		sock_ = tcpConnect(uri);
		if (sock_ != INVALID_SOCKET) {
			if (!ws_connect(sock_, uri)) {
				LOG(ERROR) << "Websocket connection failed";
				close();
			}
		} else {
			LOG(ERROR) << "Connection refused to " << uri.getHost() << ":" << uri.getPort();
		}

		status_ = kConnecting;
	} else {
		LOG(ERROR) << "Unrecognised connection protocol: " << pUri;
	}
	
	is_waiting_ = true;
	
	if (status_ == kConnecting) {
		// Install return handshake handler.
		bind("__handshake__", [this](uint64_t magic, uint32_t version, UUID pid) {
			LOG(INFO) << "Handshake 1 received";
			if (magic != ftl::net::kMagic) {
				close();
				LOG(ERROR) << "Invalid magic during handshake";
			} else {
				status_ = kConnected;
				version_ = version;
				peerid_ = pid;
				send("__handshake__", ftl::net::kMagic, ftl::net::kVersion, ftl::net::this_peer);
				
				_trigger(open_handlers_);
			}
		}); 
	}
}

void Peer::_updateURI() {
	sockaddr_storage addr;
	int rsize = sizeof(sockaddr_storage);
	if (getpeername(sock_, (sockaddr*)&addr, (socklen_t*)&rsize) == 0) {
		char addrbuf[INET6_ADDRSTRLEN];
		int port;
		
		if (addr.ss_family == AF_INET) {
			struct sockaddr_in *s = (struct sockaddr_in *)&addr;
			//port = ntohs(s->sin_port);
			inet_ntop(AF_INET, &s->sin_addr, addrbuf, INET6_ADDRSTRLEN);
			port = s->sin_port;
		} else { // AF_INET6
			struct sockaddr_in6 *s = (struct sockaddr_in6 *)&addr;
			//port = ntohs(s->sin6_port);
			inet_ntop(AF_INET6, &s->sin6_addr, addrbuf, INET6_ADDRSTRLEN);
			port = s->sin6_port;
		}
		
		// TODO verify tcp or udp etc.
		
		uri_ = std::string("tcp://")+addrbuf;
		uri_ += ":";
		uri_ += std::to_string(port);
	}
}

void Peer::close(bool retry) {
	if (sock_ != INVALID_SOCKET) {
		#ifndef WIN32
		::close(sock_);
		#else
		closesocket(sock_);
		#endif
		sock_ = INVALID_SOCKET;
		status_ = kDisconnected;

		// Attempt auto reconnect?
		
		//auto i = find(sockets.begin(),sockets.end(),this);
		//sockets.erase(i);
		
		_trigger(close_handlers_);
	}
}

/*void Peer::setProtocol(Protocol *p) {
	if (p != NULL) {
		if (proto_ == p) return;
		if (proto_ && proto_->id() == p->id()) return;
		
		if (remote_proto_ != "") {
			Handshake hs1;
			hs1.magic = ftl::net::MAGIC;
			//hs1.name_size = 0;
			hs1.proto_size = p->id().size();
			send(FTL_PROTOCOL_HS1, hs1, p->id());
			LOG(INFO) << "Handshake initiated with " << uri_;
		}
		
		proto_ = p;
	} else {
	}
}*/

void Peer::socketError() {
	int err;
#ifdef WIN32
	int optlen = sizeof(err);
#else
	uint32_t optlen = sizeof(err);
#endif
	getsockopt(sock_, SOL_SOCKET, SO_ERROR, (char*)&err, &optlen);
	LOG(ERROR) << "Socket: " << uri_ << " - error " << err;
}

void Peer::error(int e) {
	
}

void Peer::data() {
	//if (!is_waiting_) return;
	is_waiting_ = false;
	pool.push([](int id, Peer *p) {
		p->_data();
		p->is_waiting_ = true;
	}, this);
}

bool Peer::_data() {
	// std::unique_lock<std::mutex> lk(recv_mtx_);

	recv_buf_.reserve_buffer(kMaxMessage);
	int rc = ftl::net::internal::recv(sock_, recv_buf_.buffer(), kMaxMessage, 0);

	if (rc < 0) {
		return false;
	}
	
	recv_buf_.buffer_consumed(rc);
	
	msgpack::object_handle msg;
	while (recv_buf_.next(msg)) {
		msgpack::object obj = msg.get();
		if (status_ != kConnected) {
			// First message must be a handshake
			try {
				tuple<uint32_t, std::string, msgpack::object> hs;
				obj.convert(hs);
				
				if (get<1>(hs) != "__handshake__") {
					close();
					LOG(ERROR) << "Missing handshake";
					return false;
				}
			} catch(...) {
				close();
				LOG(ERROR) << "Bad first message format";
				return false;
			}
		}
		disp_->dispatch(*this, obj);
	}
	return false;
}

/*bool Socket::data() {
	//Read data from socket
	size_t n = 0;
	int c = 0;
	uint32_t len = 0;

	if (pos_ < 4) {
		n = 4 - pos_;
	} else {
		len = *(int*)buffer_;
		n = len+4-pos_;
	}

	while (pos_ < len+4) {
		if (len > MAX_MESSAGE) {
			close();
			LOG(ERROR) << "Socket: " << uri_ << " - message attack";
			return false;
		}

		const int rc = ftl::net::internal::recv(sock_, buffer_+pos_, n, 0);

		if (rc > 0) {
			pos_ += static_cast<size_t>(rc);

			if (pos_ < 4) {
				n = 4 - pos_;
			} else {
				len = *(int*)buffer_;
				n = len+4-pos_;
			}
		} else if (rc == EWOULDBLOCK || rc == 0) {
			// Data not yet available
			if (c == 0) {
				LOG(INFO) << "Socket disconnected " << uri_;
				close();
			}
			return false;
		} else {
			LOG(ERROR) << "Socket: " << uri_ << " - error " << rc;
			close();
			return false;
		}
		c++;
	}

	// Route the message...
	uint32_t service = ((uint32_t*)buffer_)[1];
	auto d = std::string(buffer_+8, len-4);
	
	pos_ = 0; // DODGY, processing messages inside handlers is dangerous.
	gpos_ = 0;
	
	if (service == FTL_PROTOCOL_HS1 && !connected_) {
		handshake1();
	} else if (service == FTL_PROTOCOL_HS2 && !connected_) {
		handshake2();
	} else if (service == FTL_PROTOCOL_RPC) {
		if (proto_) proto_->dispatchRPC(*this, d);
		else LOG(WARNING) << "No protocol set for socket " << uri_;
	} else if (service == FTL_PROTOCOL_RPCRETURN) {
		_dispatchReturn(d);
	} else {
		if (proto_) proto_->dispatchRaw(service, *this);
		else LOG(WARNING) << "No protocol set for socket " << uri_;
	}

	return true;
}*/

/*int Socket::read(char *b, size_t count) {
	if (count > size()) LOG(WARNING) << "Reading too much data for service " << header_->service;
	count = (count > size() || count==0) ? size() : count;
	// TODO, utilise recv directly here...
	memcpy(b,data_+gpos_,count);
	gpos_+=count;
	return count;
}

int Socket::read(std::string &s, size_t count) {
	count = (count > size() || count==0) ? size() : count;
	s = std::string(data_+gpos_,count);
	return count;
}

void Socket::handshake1() {
	Handshake header;
	read(header);

	std::string peer;
	if (header.name_size > 0) read(peer,header.name_size);

	std::string protouri;
	if (header.proto_size > 0) read(protouri,header.proto_size);

	if (protouri.size() > 0) {
		remote_proto_ = protouri;
		// TODO Validate protocols with local protocol?
	}

	send(FTL_PROTOCOL_HS2); // TODO Counterpart protocol.
	LOG(INFO) << "Handshake (" << protouri << ") confirmed from " << uri_;
	_connected();
}

void Socket::handshake2() {
	LOG(INFO) << "Handshake finalised for " << uri_;
	_connected();
}*/

void Peer::_dispatchResponse(uint32_t id, msgpack::object &res) {	
	// TODO Handle error reporting...
	
	if (callbacks_.count(id) > 0) {
		LOG(INFO) << "Received return RPC value";
		
		// Call the callback with unpacked return value
		(*callbacks_[id])(res);
		callbacks_.erase(id);
	} else {
		LOG(WARNING) << "Missing RPC callback for result - discarding";
	}
}

void Peer::cancelCall(int id) {
	if (callbacks_.count(id) > 0) {
		callbacks_.erase(id);
	}
}

void Peer::_sendResponse(uint32_t id, const msgpack::object &res) {
	Dispatcher::response_t res_obj = std::make_tuple(1,id,std::string(""),res);
	std::unique_lock<std::mutex> lk(send_mtx_);
	msgpack::pack(send_buf_, res_obj);
	_send();
}

void Peer::onConnect(const std::function<void(Peer&)> &f) {
	if (status_ == kConnected) {
		f(*this);
	} else {
		open_handlers_.push_back(f);
	}
}

void Peer::_connected() {
	status_ = kConnected;

}

int Peer::_send() {
	// Are we using a websocket?
	if (scheme_ == ftl::URI::SCHEME_WS) {
		// Create a websocket header as well.
		size_t len = 0;
		const iovec *sendvec = send_buf_.vector();
		size_t size = send_buf_.vector_size();
		char buf[20];  // TODO(nick) Should not be a stack buffer.
		
		// Calculate total size of message
		for (size_t i=0; i < size; i++) {
			len += sendvec[i].iov_len;
		}
		
		// Pack correct websocket header into buffer
		int rc = ws_prepare(wsheader_type::BINARY_FRAME, false, len, buf, 20);
		if (rc == -1) return -1;
		
		// Patch the first io vector to be ws header
		const_cast<iovec*>(&sendvec[0])->iov_base = buf;
		const_cast<iovec*>(&sendvec[0])->iov_len = rc;
	}
	
#ifdef WIN32
	// TODO(nick) Use WSASend instead as equivalent to writev
	auto send_vec = send_buf_.vector();
	auto send_size = send_buf_.vector_size();
	int c = 0;
	for (int i = 0; i < send_size; i++) {
		c += ftl::net::internal::send(sock_, (char*)send_vec[i].iov_base, send_vec[i].iov_len, 0);
	}
#else
	int c = ftl::net::internal::writev(sock_, send_buf_.vector(), send_buf_.vector_size());
#endif
	send_buf_.clear();
	
	// We are blocking, so -1 should mean actual error
	if (c == -1) {
		socketError();
		close();
	}
	
	return c;
}

Peer::~Peer() {
	close();

	delete disp_;
}

