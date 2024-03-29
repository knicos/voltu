/**
 * @file peer.cpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

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

#ifndef WIN32
#include <sys/ioctl.h>
#include <linux/sockios.h>
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
#include <vector>

using std::tuple;
using std::get;
using ftl::net::Peer;
using ftl::URI;
using ftl::net::ws_connect;
using ftl::net::Dispatcher;
using std::chrono::seconds;
using ftl::net::Universe;
using ftl::net::callback_t;
using std::vector;

std::atomic_int Peer::rpcid__ = 0;
std::atomic_int Peer::local_peer_ids__ = 0;

// Global peer UUID
ftl::UUID ftl::net::this_peer;

//static ctpl::thread_pool pool(5);

// TODO:(nick) Move to tcp_internal.cpp
static SOCKET tcpConnect(URI &uri, size_t ssize, size_t rsize) {
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

	int flags =1; 
    if (setsockopt(csocket, IPPROTO_TCP, TCP_NODELAY, (const char *)&flags, sizeof(flags))) { LOG(ERROR) << "ERROR: setsocketopt(), TCP_NODELAY"; };

	LOG(INFO) << "TcpConnect buffers: " << ssize << ", " << rsize;

	int a = static_cast<int>(rsize);
	if (setsockopt(csocket, SOL_SOCKET, SO_RCVBUF, (const char *)&a, sizeof(int)) == -1) {
		fprintf(stderr, "Error setting socket opts: %s\n", strerror(errno));
	}
	a = static_cast<int>(ssize);
	if (setsockopt(csocket, SOL_SOCKET, SO_SNDBUF, (const char *)&a, sizeof(int)) == -1) {
		fprintf(stderr, "Error setting socket opts: %s\n", strerror(errno));
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

	// TODO:(Nick) - Check all returned addresses.
	auto addr = addrs;
	rc = ::connect(csocket, addr->ai_addr, (socklen_t)addr->ai_addrlen);

	if (rc < 0) {
		if (errno == EINPROGRESS) {
			// FIXME:(Nick) Move to main select thread to prevent blocking
			fd_set myset;
			fd_set errset; 
			struct timeval tv;
			tv.tv_sec = 1; 
			tv.tv_usec = 0; 
			FD_ZERO(&myset); 
			FD_SET(csocket, &myset);
			FD_ZERO(&errset); 
			FD_SET(csocket, &errset); 

			rc = select(csocket+1u, NULL, &myset, &errset, &tv); 
			if (rc <= 0 || FD_ISSET(csocket, &errset)) { //} && errno != EINTR) { 
				if (rc <= 0) {
					LOG(ERROR) << "Could not connect to " << uri.getBaseURI();
				} else {
					LOG(ERROR) << "Could not connect (" << errno << ") " << uri.getBaseURI(); 	
				}

				#ifndef WIN32
				close(csocket);
				#else
				closesocket(csocket);
				#endif

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
	scheme_ = ftl::URI::SCHEME_TCP;
	outgoing_ = false;
	local_id_ = local_peer_ids__++;

	#ifndef TEST_MOCKS
	int flags =1; 
    if (setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (const char *)&flags, sizeof(flags))) { LOG(ERROR) << "ERROR: setsocketopt(), TCP_NODELAY"; };
	int a = static_cast<int>(u->getRecvBufferSize(scheme_));
	if (setsockopt(s, SOL_SOCKET, SO_RCVBUF, (const char *)&a, sizeof(int)) == -1) {
		fprintf(stderr, "Error setting socket opts: %s\n", strerror(errno));
	}
	a = static_cast<int>(u->getSendBufferSize(scheme_));
	if (setsockopt(s, SOL_SOCKET, SO_SNDBUF, (const char *)&a, sizeof(int)) == -1) {
		fprintf(stderr, "Error setting socket opts: %s\n", strerror(errno));
	}
	#endif
	
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
				ftl::pool.push([this](int id) {
					universe_->_notifyConnect(this);
				});
			}
		});

		bind("__disconnect__", [this]() {
			_badClose(false);
			LOG(INFO) << "Peer elected to disconnect: " << id().to_string();
		});

		bind("__ping__", [this]() {
			return ftl::timer::get_time();
		});

		send("__handshake__", ftl::net::kMagic, ftl::net::kVersion, ftl::net::this_peer); 
	}
}

Peer::Peer(const char *pUri, Universe *u, Dispatcher *d) : can_reconnect_(true), universe_(u), ws_read_header_(false), uri_(pUri) {	
	URI uri(pUri);
	
	status_ = kInvalid;
	sock_ = INVALID_SOCKET;
	outgoing_ = true;
	local_id_ = local_peer_ids__++;
	
	disp_ = new Dispatcher(d);

	// Must do to prevent receiving message before handlers are installed
	UNIQUE_LOCK(recv_mtx_,lk);

	scheme_ = uri.getProtocol();
	if (uri.getProtocol() == URI::SCHEME_TCP) {
		sock_ = tcpConnect(uri, u->getSendBufferSize(scheme_), u->getRecvBufferSize(scheme_));
		if (sock_ != INVALID_SOCKET) status_ = kConnecting;
		else status_ = kReconnecting;
	} else if (uri.getProtocol() == URI::SCHEME_WS) {
		LOG(INFO) << "Websocket connect " << uri.getPath();
		sock_ = tcpConnect(uri, u->getSendBufferSize(scheme_), u->getRecvBufferSize(scheme_));
		if (sock_ != INVALID_SOCKET) {
			if (!ws_connect(sock_, uri)) {
				LOG(ERROR) << "Websocket connection failed";
				_badClose(false);
			} else {
				status_ = kConnecting;
				LOG(INFO) << "Websocket connected: " << pUri;
			}
		} else {
			LOG(ERROR) << "Connection refused to " << uri.getHost() << ":" << uri.getPort();
			status_ = kReconnecting;
		}

		//status_ = kConnecting;
	} else {
		LOG(ERROR) << "Unrecognised connection protocol: " << pUri;
		return;
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
				ftl::pool.push([this](int id) {
					universe_->_notifyConnect(this);
				});
			}
		}); 

		bind("__disconnect__", [this]() {
			_badClose(false);
			LOG(INFO) << "Peer elected to disconnect: " << id().to_string();
		});

		bind("__ping__", [this]() {
			return ftl::timer::get_time();
		});
	}
}

bool Peer::reconnect() {
	if (status_ != kReconnecting || !can_reconnect_) return false;

	URI uri(uri_);

	LOG(INFO) << "Reconnecting to " << uri_ << " ...";

	if (scheme_ == URI::SCHEME_TCP) {
		sock_ = tcpConnect(uri, universe_->getSendBufferSize(scheme_), universe_->getRecvBufferSize(scheme_));
		if (sock_ != INVALID_SOCKET) {
			status_ = kConnecting;
			is_waiting_ = true;
			return true;
		} else {
			return false;
		}
	} else if (scheme_ == URI::SCHEME_WS) {
		sock_ = tcpConnect(uri, universe_->getSendBufferSize(scheme_), universe_->getRecvBufferSize(scheme_));
		if (sock_ != INVALID_SOCKET) {
			if (!ws_connect(sock_, uri)) {
				return false;
			} else {
				status_ = kConnecting;
				LOG(INFO) << "WEB SOCK CONNECTED";
				return true;
			}
		} else {
			return false;
		}
	}

	// TODO:(Nick) allow for other protocols in reconnect
	return false;
}

void Peer::_updateURI() {
	sockaddr_storage addr;

	// FIXME:(Nick) Get actual protocol...
	scheme_ = ftl::URI::SCHEME_TCP;

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
		LOG(INFO) << "Deliberate disconnect of peer: " << uri_;
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

		universe_->_notifyDisconnect(this);
		status_ = kDisconnected;

		// Attempt auto reconnect?
		if (retry && can_reconnect_) {
			status_ = kReconnecting;
		}
	}
}

bool Peer::socketError() {
	int err;
#ifdef WIN32
	int optlen = sizeof(err);
#else
	uint32_t optlen = sizeof(err);
#endif
	getsockopt(sock_, SOL_SOCKET, SO_ERROR, (char*)&err, &optlen);

	if (err == 0) return false;

	// Must close before log since log may try to send over net causing
	// more socket errors...
	_badClose();

	LOG(ERROR) << "Socket: " << uri_ << " - error " << err;
	return true;
}

void Peer::error(int e) {
	
}

void Peer::data() {
	{
		UNIQUE_LOCK(recv_mtx_,lk);

		int rc=0;

		recv_buf_.reserve_buffer(kMaxMessage);

		if (recv_buf_.buffer_capacity() < (kMaxMessage / 10)) {
			LOG(WARNING) << "Net buffer at capacity";
			return;
		}

		int cap = static_cast<int>(recv_buf_.buffer_capacity());
		auto buf = recv_buf_.buffer();
		lk.unlock();

		rc = ftl::net::internal::recv(sock_, buf, cap, 0);

		if (rc >= cap-1) {
			LOG(WARNING) << "More than buffers worth of data received"; 
		}
		if (cap < (kMaxMessage / 10)) LOG(WARNING) << "NO BUFFER";

		if (rc == 0) {
			close();
			return;
		} else if (rc < 0) {
			socketError();
			return;
		}
		
		lk.lock();
		recv_buf_.buffer_consumed(rc);

		if (is_waiting_) {
			is_waiting_ = false;
			lk.unlock();

			ftl::pool.push([this](int id) {
				_data();
			});
		}
	}
}

bool Peer::_data() {
	msgpack::object_handle msg;

	UNIQUE_LOCK(recv_mtx_,lk);

	if (scheme_ == ftl::URI::SCHEME_WS && !ws_read_header_) {
		//LOG(INFO) << "Reading WS Header";
		wsheader_type ws;
		ws.header_size = 0;
		if (ws_parse(recv_buf_, ws) < 0) {
			//LOG(ERROR) << "Bad WS header " << ws.header_size;
			is_waiting_ = true;
			return false;
		}
		ws_read_header_ = true;
	}

	if (!recv_buf_.next(msg)) {
		is_waiting_ = true;
		return false;
	}
	ws_read_header_ = false;
	
	lk.unlock();

	msgpack::object obj = msg.get();
	ftl::pool.push([this](int id) {
		_data();
	});

	if (status_ == kConnecting) {
		// If not connected, must lock to make sure no other thread performs this step
		UNIQUE_LOCK(recv_mtx_,lk);
		// Verify still not connected after lock
		if (status_ == kConnecting) {
			// First message must be a handshake
			try {
				tuple<uint32_t, std::string, msgpack::object> hs;
				obj.convert(hs);
				
				if (get<1>(hs) != "__handshake__") {
					LOG(WARNING) << "Missing handshake - got '" << get<1>(hs) << "'";

					// Allow a small delay in case another thread is doing the handshake
					lk.unlock();
					std::this_thread::sleep_for(std::chrono::milliseconds(10));
					if (status_ == kConnecting) {
						LOG(ERROR) << "Failed to get handshake";
						_badClose(false);
						return false;
					}
				} else {
					// Must handle immediately with no other thread able
					// to read next message before completion.
					// The handshake handler must not block.
					disp_->dispatch(*this, obj);
					return true;
				}
			} catch(...) {
				LOG(WARNING) << "Bad first message format... waiting";

				// Allow a small delay in case another thread is doing the handshake
				lk.unlock();
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
				if (status_ == kConnecting) {
					LOG(ERROR) << "Failed to get handshake";
					_badClose(false);
					return false;
				}
			}
		}
	}
	
	disp_->dispatch(*this, obj);

	// Lock again before freeing msg handle
	UNIQUE_LOCK(recv_mtx_,lk2);
	return true;
}

void Peer::_dispatchResponse(uint32_t id, const std::string &name, msgpack::object &res) {	
	// TODO: Handle error reporting...
	UNIQUE_LOCK(cb_mtx_,lk);
	if (callbacks_.count(id) > 0) {
		
		// Allow for unlock before callback
		auto cb = std::move(callbacks_[id]);
		callbacks_.erase(id);
		lk.unlock();

		// Call the callback with unpacked return value
		try {
			(*cb)(res);
		} catch(std::exception &e) {
			LOG(ERROR) << "Exception in RPC response: " << e.what();
		}
	} else {
		LOG(WARNING) << "Missing RPC callback for result - discarding: " << name;
	}
}

void Peer::cancelCall(int id) {
	UNIQUE_LOCK(cb_mtx_,lk);
	if (callbacks_.count(id) > 0) {
		callbacks_.erase(id);
	}
}

void Peer::_sendResponse(uint32_t id, const std::string &name, const msgpack::object &res) {
	Dispatcher::response_t res_obj = std::make_tuple(1,id,name,res);
	UNIQUE_LOCK(send_mtx_,lk);
	if (scheme_ == ftl::URI::SCHEME_WS) send_buf_.append_ref(nullptr,0);
	msgpack::pack(send_buf_, res_obj);
	_send();
}

void Peer::_waitCall(int id, std::condition_variable &cv, bool &hasreturned, const std::string &name) {
	std::mutex m;

	int64_t beginat = ftl::timer::get_time();
	std::function<void(int)> j;
	while (!hasreturned) {
		// Attempt to do a thread pool job if available
		if ((bool)(j=ftl::pool.pop())) {
			j(-1);
		} else {
			// Block for a little otherwise
			std::unique_lock<std::mutex> lk(m);
			cv.wait_for(lk, std::chrono::milliseconds(2), [&hasreturned]{return hasreturned;});
		}

		if (ftl::timer::get_time() - beginat > 1000) break;
	}
	
	if (!hasreturned) {
		cancelCall(id);
		throw FTL_Error("RPC failed with timeout: " << name);
	}
}

bool Peer::waitConnection() {
	if (status_ == kConnected) return true;
	else if (status_ != kConnecting) return false;
	
	std::mutex m;
	//UNIQUE_LOCK(m,lk);
	std::unique_lock<std::mutex> lk(m);
	std::condition_variable cv;

	callback_t h = universe_->onConnect([this,&cv](Peer *p) {
		if (p == this) {
			cv.notify_one();
		}
	});

	cv.wait_for(lk, seconds(1), [this](){return status_ == kConnected;});
	universe_->removeCallback(h);
	return status_ == kConnected;
}

void Peer::_connected() {
	status_ = kConnected;

}

int Peer::_send() {
	if (sock_ == INVALID_SOCKET) return -1;

	int c=0;

	// Are we using a websocket?
	if (scheme_ == ftl::URI::SCHEME_WS) {
		// Create a websocket header as well.
		size_t len = 0;
		const iovec *sendvec = send_buf_.vector();
		size_t size = send_buf_.vector_size();
		char buf[20];

		const uint8_t masking_key[4] = { 0x12, 0x34, 0x56, 0x78 }; // TODO: Move
		
		// Calculate total size of message and mask it.
		for (size_t i=1; i < size; i++) {
			const size_t mlen = sendvec[i].iov_len;
			char *buf = (char*)sendvec[i].iov_base;
			// TODO: Make this more efficient.
			for (size_t j = 0; j != mlen; ++j) {
                buf[j] ^= masking_key[(len + j)&0x3];
            }
			len += mlen;
		}

		if (sendvec[0].iov_len != 0) {
			LOG(FATAL) << "CORRUPTION in websocket header buffer";
		}
		
		// Pack correct websocket header into buffer
		int rc = ws_prepare(wsheader_type::BINARY_FRAME, true, len, buf, 20);
		if (rc == -1) return -1;
		
		// Patch the first io vector to be ws header
		const_cast<iovec*>(&sendvec[0])->iov_base = buf;
		const_cast<iovec*>(&sendvec[0])->iov_len = rc;
	
#ifdef WIN32
		auto send_vec = send_buf_.vector();
		auto send_size = send_buf_.vector_size();
		vector<WSABUF> wsabuf(send_size);

		for (int i = 0; i < send_size; i++) {
			wsabuf[i].len = (ULONG)send_vec[i].iov_len;
			wsabuf[i].buf = (char*)send_vec[i].iov_base;
			//c += ftl::net::internal::send(sock_, (char*)send_vec[i].iov_base, (int)send_vec[i].iov_len, 0);
		}

		DWORD bytessent;
		c = ftl::net::internal::writev(sock_, wsabuf.data(), static_cast<DWORD>(send_size), (LPDWORD)&bytessent);
#else
		c = ftl::net::internal::writev(sock_, send_buf_.vector(), (int)send_buf_.vector_size());
#endif

	} else {
#ifdef WIN32
		auto send_vec = send_buf_.vector();
		auto send_size = send_buf_.vector_size();
		vector<WSABUF> wsabuf(send_size);

		for (int i = 0; i < send_size; i++) {
			wsabuf[i].len = (ULONG)send_vec[i].iov_len;
			wsabuf[i].buf = (char*)send_vec[i].iov_base;
			//c += ftl::net::internal::send(sock_, (char*)send_vec[i].iov_base, (int)send_vec[i].iov_len, 0);
		}

		DWORD bytessent;
		c = ftl::net::internal::writev(sock_, wsabuf.data(), static_cast<DWORD>(send_size), (LPDWORD)&bytessent);
#else
		c = ftl::net::internal::writev(sock_, send_buf_.vector(), (int)send_buf_.vector_size());
#endif
	} 

	send_buf_.clear();
	
	// We are blocking, so -1 should mean actual error
	if (c == -1) {
		socketError();
		//_badClose();
	}
	
	return c;
}

Peer::~Peer() {
	UNIQUE_LOCK(send_mtx_,lk1);
	UNIQUE_LOCK(recv_mtx_,lk2);
	_badClose(false);
	LOG(INFO) << "Deleting peer object";

	delete disp_;
}

