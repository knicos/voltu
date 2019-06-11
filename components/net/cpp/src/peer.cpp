//#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <loguru.hpp>
#include <ctpl_stl.h>

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <ftl/net/common.hpp>

#include <fcntl.h>
#ifdef WIN32
#include <Ws2tcpip.h>
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
#include <ftl/net/universe.hpp>

#include <iostream>
#include <memory>
#include <algorithm>
#include <tuple>
#include <chrono>

using std::tuple;
using std::get;
using ftl::net::Peer;
using ftl::URI;
using ftl::net::ws_connect;
using ftl::net::Dispatcher;
using std::chrono::seconds;
using ftl::net::Universe;
using ftl::net::callback_t;
using std::mutex;
using std::recursive_mutex;
using std::unique_lock;

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
static SOCKET tcpConnect(URI &uri) {
	int rc;
	//sockaddr_in destAddr;

	#ifdef WIN32
	WSAData wsaData;
	if (WSAStartup(MAKEWORD(1,1), &wsaData) != 0) {
		LOG(ERROR) << "Could not initiate sockets";
		return INVALID_SOCKET;
	}
	#endif
	
	//We want a TCP socket
	SOCKET csocket = socket(AF_INET, SOCK_STREAM, 0);

	if (csocket == INVALID_SOCKET) {
		LOG(ERROR) << "Unable to create TCP socket";
		return INVALID_SOCKET;
	}

	addrinfo hints = {}, *addrs;
	hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
	rc = getaddrinfo(uri.getHost().c_str(), std::to_string(uri.getPort()).c_str(), &hints, &addrs);

	if (rc != 0 || addrs == nullptr) {
		#ifndef WIN32
		close(csocket);
		#else
		closesocket(csocket);
		#endif

		LOG(ERROR) << "Address not found : " << uri.getHost() << std::endl;
		return INVALID_SOCKET;
	}

	// Make nonblocking
#ifndef WIN32
	long arg = fcntl(csocket, F_GETFL, NULL);
	arg |= O_NONBLOCK;
	fcntl(csocket, F_SETFL, arg);
#endif

	// TODO(Nick) - Check all returned addresses.
	auto addr = addrs;
	rc = ::connect(csocket, addr->ai_addr, (socklen_t)addr->ai_addrlen);

	if (rc < 0) {
		if (errno == EINPROGRESS) {
			fd_set myset; 
			struct timeval tv;
			tv.tv_sec = 1; 
			tv.tv_usec = 0; 
			FD_ZERO(&myset); 
			FD_SET(csocket, &myset); 
			rc = select(csocket+1, NULL, &myset, NULL, &tv); 
			if (rc <= 0) { //} && errno != EINTR) { 
				#ifndef WIN32
				close(csocket);
				#else
				closesocket(csocket);
				#endif

				LOG(ERROR) << "Could not connect to " << uri.getBaseURI();

				return INVALID_SOCKET;
			}
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
#ifndef WIN32
	arg = fcntl(csocket, F_GETFL, NULL);
	arg &= (~O_NONBLOCK);
	fcntl(csocket, F_SETFL, arg);
#endif

	return csocket;
}

Peer::Peer(SOCKET s, Universe *u, Dispatcher *d) : sock_(s), can_reconnect_(false), universe_(u), ws_read_header_(false) {
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
				_badClose(false);
				LOG(ERROR) << "Invalid magic during handshake";
			} else {
				status_ = kConnected;
				version_ = version;
				peerid_ = pid;
				if (version != ftl::net::kVersion) LOG(WARNING) << "Net protocol using different versions!";
				
				// Ensure handlers called later or in new thread
				pool.push([this](int id) {
					universe_->_notifyConnect(this);
				});
			}
		});

		bind("__disconnect__", [this]() {
			_badClose(false);
			LOG(INFO) << "Peer elected to disconnect: " << id().to_string();
		});

		bind("__ping__", [this](unsigned long long timestamp) {
			return timestamp;
		});

		send("__handshake__", ftl::net::kMagic, ftl::net::kVersion, ftl::net::this_peer); 
	}
}

Peer::Peer(const char *pUri, Universe *u, Dispatcher *d) : can_reconnect_(true), universe_(u), ws_read_header_(false), uri_(pUri) {	
	URI uri(pUri);
	
	status_ = kInvalid;
	sock_ = INVALID_SOCKET;
	
	disp_ = new Dispatcher(d);

	// Must to to prevent receiving message before handlers are installed
	unique_lock<recursive_mutex> lk(recv_mtx_);

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
				_badClose(false);
			} else {
				status_ = kConnecting;
				LOG(INFO) << "WEB SOCK CONNECTED";
			}
		} else {
			LOG(ERROR) << "Connection refused to " << uri.getHost() << ":" << uri.getPort();
		}

		status_ = kConnecting;
	} else {
		LOG(ERROR) << "Unrecognised connection protocol: " << pUri;
	}
	
	is_waiting_ = true;
	
	if (status_ == kConnecting || status_ == kReconnecting) {
		// Install return handshake handler.
		bind("__handshake__", [this](uint64_t magic, uint32_t version, UUID pid) {
			LOG(INFO) << "Handshake 1 received";
			if (magic != ftl::net::kMagic) {
				_badClose(false);
				LOG(ERROR) << "Invalid magic during handshake";
			} else {
				status_ = kConnected;
				version_ = version;
				peerid_ = pid;
				if (version != ftl::net::kVersion) LOG(WARNING) << "Net protocol using different versions!";
				send("__handshake__", ftl::net::kMagic, ftl::net::kVersion, ftl::net::this_peer);
				
				// Ensure handlers called later or in new thread
				pool.push([this](int id) {
					universe_->_notifyConnect(this);
				});
			}
		}); 

		bind("__disconnect__", [this]() {
			_badClose(false);
			LOG(INFO) << "Peer elected to disconnect: " << id().to_string();
		});

		bind("__ping__", [this](unsigned long long timestamp) {
			return timestamp;
		});
	}
}

bool Peer::reconnect() {
	if (status_ != kReconnecting || !can_reconnect_) return false;

	URI uri(uri_);

	LOG(INFO) << "Reconnecting to " << uri_ << " ...";

	if (scheme_ == URI::SCHEME_TCP) {
		sock_ = tcpConnect(uri);
		if (sock_ != INVALID_SOCKET) {
			status_ = kConnecting;
			is_waiting_ = true;
			return true;
		} else {
			return false;
		}
	}

	// TODO(Nick) allow for other protocols in reconnect
	return false;
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

		// Attempt to inform about disconnect
		send("__disconnect__");

		_badClose(retry);
		LOG(INFO) << "Deliberate disconnect of peer.";
	}
}

void Peer::_badClose(bool retry) {
	if (sock_ != INVALID_SOCKET) {
		#ifndef WIN32
		::close(sock_);
		#else
		closesocket(sock_);
		#endif
		sock_ = INVALID_SOCKET;
		status_ = kDisconnected;
		
		//auto i = find(sockets.begin(),sockets.end(),this);
		//sockets.erase(i);

		universe_->_notifyDisconnect(this);

		// Attempt auto reconnect?
		if (retry && can_reconnect_) {
			status_ = kReconnecting;
		}
	}
}

void Peer::socketError() {
	int err;
#ifdef WIN32
	int optlen = sizeof(err);
#else
	uint32_t optlen = sizeof(err);
#endif
	getsockopt(sock_, SOL_SOCKET, SO_ERROR, (char*)&err, &optlen);

	// Must close before log since log may try to send over net causing
	// more socket errors...
	_badClose();

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

inline std::ostream& hex_dump(std::ostream& o, std::string const& v) {
    std::ios::fmtflags f(o.flags());
    o << std::hex;
    for (auto c : v) {
        o << "0x" << std::setw(2) << std::setfill('0') << (static_cast<int>(c) & 0xff) << ' ';
    }
    o.flags(f);
    return o;
}

bool Peer::_data() {
	std::unique_lock<std::recursive_mutex> lk(recv_mtx_);

	recv_buf_.reserve_buffer(kMaxMessage);
	int rc = ftl::net::internal::recv(sock_, recv_buf_.buffer(), kMaxMessage, 0);

	if (rc <= 0) {
		return false;
	}
	
	recv_buf_.buffer_consumed(rc);

	if (scheme_ == ftl::URI::SCHEME_WS && !ws_read_header_) {
		wsheader_type ws;
		if (ws_parse(recv_buf_, ws) < 0) {
			return false;
		}
		ws_read_header_ = true;
	}

	/*if (rc > 0) {
		hex_dump(std::cout, std::string((char*)recv_buf_.nonparsed_buffer(), recv_buf_.nonparsed_size()));
		std::cout << std::endl;
	}*/

	msgpack::object_handle msg;
	while (recv_buf_.next(msg)) {
		ws_read_header_ = false;
		msgpack::object obj = msg.get();
		if (status_ != kConnected) {
			// First message must be a handshake
			try {
				tuple<uint32_t, std::string, msgpack::object> hs;
				obj.convert(hs);
				
				if (get<1>(hs) != "__handshake__") {
					_badClose(false);
					LOG(ERROR) << "Missing handshake - got '" << get<1>(hs) << "'";
					return false;
				}
			} catch(...) {
				_badClose(false);
				LOG(ERROR) << "Bad first message format";
				return false;
			}
		}
		disp_->dispatch(*this, obj);

		if (recv_buf_.nonparsed_size() > 0 && scheme_ == ftl::URI::SCHEME_WS) {
			wsheader_type ws;
			if (ws_parse(recv_buf_, ws) < 0) {
				return false;
			}
			ws_read_header_ = true;	
		}
	}
	return false;
}

void Peer::_dispatchResponse(uint32_t id, msgpack::object &res) {	
	// TODO Handle error reporting...
	
	if (callbacks_.count(id) > 0) {
		DLOG(1) << "Received return RPC value";
		
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
	std::unique_lock<std::recursive_mutex> lk(send_mtx_);
	if (scheme_ == ftl::URI::SCHEME_WS) send_buf_.append_ref(nullptr,0);
	msgpack::pack(send_buf_, res_obj);
	_send();
}

bool Peer::waitConnection() {
	if (status_ == kConnected) return true;
	
	std::mutex m;
	std::unique_lock<std::mutex> lk(m);
	std::condition_variable cv;

	callback_t h = universe_->onConnect([this,&cv](Peer *p) {
		if (p == this) {
			cv.notify_one();
		}
	});

	cv.wait_for(lk, seconds(5));
	universe_->removeCallback(h);
	return status_ == kConnected;
}

/*void Peer::onConnect(const std::function<void(Peer&)> &f) {
	if (status_ == kConnected) {
		f(*this);
	} else {
		open_handlers_.push_back(f);
	}
}*/

void Peer::_connected() {
	status_ = kConnected;

}

int Peer::_send() {
	if (sock_ == INVALID_SOCKET) return -1;

	// Are we using a websocket?
	if (scheme_ == ftl::URI::SCHEME_WS) {
		// Create a websocket header as well.
		size_t len = 0;
		const iovec *sendvec = send_buf_.vector();
		size_t size = send_buf_.vector_size();
		char buf[20];  // TODO(nick) Should not be a stack buffer.
		
		// Calculate total size of message
		for (size_t i=1; i < size; i++) {
			len += sendvec[i].iov_len;
		}

		//LOG(INFO) << "SEND SIZE = " << len;
		
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
		c += ftl::net::internal::send(sock_, (char*)send_vec[i].iov_base, (int)send_vec[i].iov_len, 0);
	}
#else
	int c = ftl::net::internal::writev(sock_, send_buf_.vector(), (int)send_buf_.vector_size());
#endif
	send_buf_.clear();
	
	// We are blocking, so -1 should mean actual error
	if (c == -1) {
		socketError();
		//_badClose();
	}
	
	return c;
}

Peer::~Peer() {
	std::unique_lock<std::recursive_mutex> lk1(send_mtx_);
	std::unique_lock<std::recursive_mutex> lk2(recv_mtx_);
	_badClose(false);
	LOG(INFO) << "Deleting peer object";

	delete disp_;
}

