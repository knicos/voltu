#include <glog/logging.h>

#include <ftl/uri.hpp>
#include <ftl/net/socket.hpp>

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

#ifdef WIN32
#include <windows.h>
#include <winsock.h>
typedef int socklen_t;
#define MSG_WAITALL 0
#endif

#include <iostream>
#include <memory>
#include <algorithm>

using namespace ftl;
using ftl::net::Socket;
using namespace std;

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

int Socket::rpcid__ = 0;

static int tcpConnect(URI &uri) {
	int rc;
	sockaddr_in destAddr;

	//std::cerr << "TCP Connect: " << uri.getHost() << " : " << uri.getPort() << std::endl;

	#ifdef WIN32
	WSAData wsaData;
	if (WSAStartup(MAKEWORD(1,1), &wsaData) != 0) {
		//ERROR
		return INVALID_SOCKET;
	}
	#endif
	
	//We want a TCP socket
	int csocket = socket(AF_INET, SOCK_STREAM, 0);

	if (csocket == INVALID_SOCKET) {
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
	/*rg = fcntl(csocket, F_GETFL, NULL));
	arg &= (~O_NONBLOCK);
	fcntl(csocket, F_SETFL, arg) < 0)*/
	
	// Handshake??

	return csocket;
}

static int wsConnect(URI &uri) {
	return 1;
}

Socket::Socket(int s) : sock_(s), pos_(0), proto_(nullptr) {
	valid_ = true;
	
	buffer_ = new char[BUFFER_SIZE];
	header_ = (Header*)buffer_;
	data_ = buffer_+sizeof(Header);
	buffer_w_ = new char[BUFFER_SIZE];
	header_w_ = (Header*)buffer_w_;
	
	connected_ = false;
	
	_updateURI();
}

Socket::Socket(const char *pUri) : pos_(0), uri_(pUri), proto_(nullptr) {
	// Allocate buffer
	buffer_ = new char[BUFFER_SIZE];
	header_ = (Header*)buffer_;
	data_ = buffer_+sizeof(Header);
	buffer_w_ = new char[BUFFER_SIZE];
	header_w_ = (Header*)buffer_w_;
	
	URI uri(pUri);
	
	valid_ = false;
	connected_ = false;
	sock_ = INVALID_SOCKET;

	if (uri.getProtocol() == URI::SCHEME_TCP) {
		sock_ = tcpConnect(uri);
		valid_ = true;
	} else if (uri.getProtocol() == URI::SCHEME_WS) {
		wsConnect(uri);
		LOG(ERROR) << "Websocket currently unsupported";
	} else {
		LOG(ERROR) << "Unrecognised connection protocol: " << pUri;
	}
}

void Socket::_updateURI() {
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

int Socket::close() {
	if (sock_ != INVALID_SOCKET) {
		#ifndef WIN32
		::close(sock_);
		#else
		closesocket(sock_);
		#endif
		sock_ = INVALID_SOCKET;
		connected_ = false;

		// Attempt auto reconnect?
		
		//auto i = find(sockets.begin(),sockets.end(),this);
		//sockets.erase(i);
	}
	return 0;
}

void Socket::setProtocol(Protocol *p) {
	if (proto_ == p) return;
	if (proto_ && proto_->id() == p->id()) return;
	
	proto_ = p;
	ftl::net::Handshake hs1;
	hs1.proto = p->id();
	send(FTL_PROTOCOL_HS1, std::string((char*)&hs1, sizeof(hs1)));
	LOG(INFO) << "Handshake initiated with " << uri_;
}

void Socket::error() {
	int err;
	uint32_t optlen = sizeof(err);
	getsockopt(sock_, SOL_SOCKET, SO_ERROR, &err, &optlen);
	LOG(ERROR) << "Socket: " << uri_ << " - error " << err;
}

bool Socket::data() {
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

		const int rc = recv(sock_, buffer_+pos_, n, 0);

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
		handshake1(d);
	} else if (service == FTL_PROTOCOL_HS2 && !connected_) {
		handshake2(d);
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
}

int Socket::read(char *b, size_t count) {
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

void Socket::handshake1(const std::string &d) {
	ftl::net::Handshake *hs;
	if (d.size() != sizeof(ftl::net::Handshake)) {
		LOG(ERROR) << "Handshake failed for " << uri_;
		close();
		return;
	}
	
	hs = (ftl::net::Handshake*)d.data();
	auto proto = Protocol::find(hs->proto);
	if (proto == NULL) {
		LOG(ERROR) << "Protocol (" << hs->proto << ") not found during handshake for " << uri_;
		close();
		return;
	} else {
		proto_ = proto;
	}
	peerid_ = std::string(&hs->peerid[0],16);
	
	ftl::net::Handshake hs2;
	//hs2.magic = ftl::net::MAGIC;
	//hs2.version = version_;
	// TODO Set peerid;
	send(FTL_PROTOCOL_HS2, std::string((char*)&hs2, sizeof(hs2)));
	LOG(INFO) << "Handshake" << " confirmed from " << uri_;
	_connected();
}

void Socket::handshake2(const std::string &d) {
	ftl::net::Handshake *hs;
	if (d.size() != sizeof(ftl::net::Handshake)) {
		LOG(ERROR) << "Handshake failed for " << uri_;
		close();
		return;
	}
	
	hs = (ftl::net::Handshake*)d.data();
	/*if (hs->magic != ftl::net::MAGIC) {
		LOG(ERROR) << "Handshake magic failed for " << uri_;
		close();
		return;
	}
	
	version_ = (hs->version > ftl::net::version()) ?
			ftl::net::version() :
			hs->version;*/
	peerid_ = std::string(&hs->peerid[0],16);
	LOG(INFO) << "Handshake finalised for " << uri_;
	_connected();
}

void Socket::_dispatchReturn(const std::string &d) {
	auto unpacked = msgpack::unpack(d.data(), d.size());
	Dispatcher::response_t the_result;
	unpacked.get().convert(the_result);

	if (std::get<0>(the_result) != 1) {
		LOG(ERROR) << "Bad RPC return message";
		return;
	}

	auto &&id = std::get<1>(the_result);
	//auto &&err = std::get<2>(the_result);
	auto &&res = std::get<3>(the_result);
	
	// TODO Handle error reporting...
	
	if (callbacks_.count(id) > 0) {
		LOG(INFO) << "Received return RPC value";
		(*callbacks_[id])(res);
		callbacks_.erase(id);
	} else {
		LOG(ERROR) << "Missing RPC callback for result";
	}
}

void Socket::onConnect(std::function<void(Socket&)> f) {
	if (connected_) {
		f(*this);
	} else {
		connect_handlers_.push_back(f);
	}
}

void Socket::_connected() {
	connected_ = true;
	for (auto h : connect_handlers_) {
		h(*this);
	}
	//connect_handlers_.clear();
}

int Socket::_send() {
	int c = ::writev(sock_, send_vec_.data(), send_vec_.size());
	send_vec_.clear();
	return c;
}

Socket::~Socket() {
	std::cerr << "DESTROYING SOCKET" << std::endl;
	close();
	
	// Delete socket buffer
	if (buffer_) delete [] buffer_;
	buffer_ = NULL;
	if (buffer_w_) delete [] buffer_w_;
	buffer_w_ = NULL;
}

