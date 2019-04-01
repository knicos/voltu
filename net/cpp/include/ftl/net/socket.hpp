#ifndef _FTL_NET_SOCKET_HPP_
#define _FTL_NET_SOCKET_HPP_

#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <glog/logging.h>
#include <ftl/net.hpp>
#include <ftl/net/protocol.hpp>

#ifndef WIN32
#define INVALID_SOCKET -1
#include <netinet/in.h>
#endif

#ifdef WIN32
//#include <windows.h>
//#include <winsock.h>
#include <winsock2.h>
#endif

#include <iostream>
#include <sstream>
#include <tuple>
#include <vector>
#include <type_traits>

# define ENABLE_IF(...) \
  typename std::enable_if<(__VA_ARGS__), bool>::type = true

extern bool _run(bool blocking, bool nodelay);

namespace ftl {
namespace net {

struct virtual_caller {
	virtual void operator()(msgpack::object &o)=0;
};

template <typename T>
struct caller : virtual_caller {
	explicit caller(std::function<void(const T&)> &f) : f_(f) {};
	void operator()(msgpack::object &o) { T r = o.as<T>(); f_(r); };
	std::function<void(const T&)> f_;
};

typedef std::tuple<const char*,size_t> array;
/*struct compress{};
struct encrypt{};
struct decompress{};
struct decrypt{};*/

/**
 * A single socket connection object, to be constructed using the connect()
 * function and not to be created directly.
 */
class Socket {
	public:
	friend bool ::_run(bool blocking, bool nodelay);
	public:
	explicit Socket(const char *uri);
	explicit Socket(int s);
	~Socket();
	
	int close();
	
	void setProtocol(Protocol *p);
	Protocol *protocol() const { return proto_; }

	/**
	 * Get the internal OS dependent socket.
	 */
	int _socket() const { return sock_; };

	bool isConnected() const { return sock_ != INVALID_SOCKET && connected_; };
	bool isValid() const { return valid_ && sock_ != INVALID_SOCKET; };
	
	/**
	 * Get the sockets protocol, address and port as a url string. This will be
	 * the same as the initial connection string on the client.
	 */
	std::string getURI() const { return uri_; };
			
	/**
	 * Non-blocking Remote Procedure Call using a callback function.
	 */
	template <typename T, typename... ARGS>
	void asyncCall(const std::string &name,
			std::function<void(const T&)> cb,
			ARGS... args);
	
	/**
	 * Blocking Remote Procedure Call.
	 */
	template <typename R, typename... ARGS>
	R call(const std::string &name, ARGS... args);

	template <typename... ARGS>
	int send(uint32_t s, ARGS... args);
	
	void begin(uint32_t s);
	
	template <typename T>
	Socket &operator<<(T &t);
	
	void end();
	
	template <typename T>
	int read(T *b, size_t count=1);

	int read(char *b, size_t count);
	int read(std::string &s, size_t count=0);
	
	template <typename T>
	int read(std::vector<T> &b, size_t count=0);
	
	template <typename T>
	int read(T &b);
	
	template <typename T>
	Socket &operator>>(T &t);
	
	//SocketStream stream(uint32_t service);
	
	size_t size() const { return header_->size-4; }

	void onError(std::function<void(Socket&, int err, const char *msg)> &f) {}
	void onConnect(std::function<void(Socket&)> &f);
	void onDisconnect(std::function<void(Socket&)> &f) {}
	
	protected:
	bool data();	// Process one message from socket
	void error();	// Process one error from socket
	
	/**
	 * Internal handlers for specific event types. This should be private but
	 * is current here for testing purposes.
	 * @{
	 */
	void handshake1();
	void handshake2();
	/** @} */
	
	private: // Functions
	void _connected();
	void _updateURI();
	void _dispatchReturn(const std::string &d);
	
	int _send();
	
	template <typename... ARGS>
	int _send(const std::string &t, ARGS... args);
	
	template <typename... ARGS>
	int _send(const array &b, ARGS... args);
	
	template <typename T, typename... ARGS>
	int _send(const std::vector<T> &t, ARGS... args);
	
	template <typename... Types, typename... ARGS>
	int _send(const std::tuple<Types...> &t, ARGS... args);
	
	template <typename T, typename... ARGS,
			ENABLE_IF(std::is_trivial<T>::value && !std::is_pointer<T>::value)>
	int _send(const T &t, ARGS... args);

	private: // Data
	bool valid_;
	bool connected_;
	int sock_;
	
	// Receive buffers
	size_t pos_;
	size_t gpos_;
	char *buffer_;
	ftl::net::Header *header_;
	char *data_;
	
	// Send buffers
	char *buffer_w_;
	std::vector<iovec> send_vec_;
	ftl::net::Header *header_w_;
	
	std::string uri_;
	std::string peerid_;
	std::string remote_proto_;

	Protocol *proto_; 
	
	std::vector<std::function<void(Socket&)>> connect_handlers_;
	std::map<int, std::unique_ptr<virtual_caller>> callbacks_;
	
	static int rpcid__;

	static const int MAX_MESSAGE = 10*1024*1024; // 10Mb currently
	static const int BUFFER_SIZE = MAX_MESSAGE + 16;
};

// --- Inline Template Implementations -----------------------------------------

template <typename... ARGS>
int Socket::send(uint32_t s, ARGS... args) {
	header_w_->service = s;
	header_w_->size = 4;
	send_vec_.push_back({header_w_,sizeof(ftl::net::Header)});
	return _send(args...);
}

template <typename T>
int Socket::read(T *b, size_t count) {
	static_assert(std::is_trivial<T>::value, "Can only read trivial types");
	return read((char*)b, sizeof(T)*count);
}

template <typename T>
int Socket::read(std::vector<T> &b, size_t count) {
	count = (count == 0) ? size()/sizeof(T) : count; // TODO Round this!
	if (b.size() != count) b.resize(count);
	return read((char*)b.data(), sizeof(T)*count);
}

template <typename T>
int Socket::read(T &b) {
	if (std::is_array<T>::value) return read(&b,std::extent<T>::value);
	else return read(&b);
}

template <typename T>
Socket &Socket::operator>>(T &t) {
	if (std::is_array<T>::value) read(&t,std::extent<T>::value);
	else read(&t);
	return *this;
}

template <typename... ARGS>
int Socket::_send(const std::string &t, ARGS... args) {
	send_vec_.push_back({const_cast<char*>(t.data()),t.size()});
	header_w_->size += t.size();
	return _send(args...)+t.size();
}

template <typename... ARGS>
int Socket::_send(const ftl::net::array &b, ARGS... args) {
	send_vec_.push_back({const_cast<char*>(std::get<0>(b)),std::get<1>(b)});
	header_w_->size += std::get<1>(b);
	return std::get<1>(b)+_send(args...);
}

template <typename T, typename... ARGS>
int Socket::_send(const std::vector<T> &t, ARGS... args) {
	send_vec_.push_back({const_cast<char*>(t.data()),t.size()});
	header_w_->size += t.size();
	return _send(args...)+t.size();
}

template <typename... Types, typename... ARGS>
int Socket::_send(const std::tuple<Types...> &t, ARGS... args) {
	send_vec_.push_back({const_cast<char*>((char*)&t),sizeof(t)});
	header_w_->size += sizeof(t);
	return sizeof(t)+_send(args...);
}

template <typename T, typename... ARGS,
		ENABLE_IF(std::is_trivial<T>::value && !std::is_pointer<T>::value)>
int Socket::_send(const T &t, ARGS... args) {
	send_vec_.push_back({const_cast<T*>(&t),sizeof(T)});
	header_w_->size += sizeof(T);
	return sizeof(T)+_send(args...);
}

//template <typename T, typename... ARGS>
template <typename R, typename... ARGS>
R Socket::call(const std::string &name, ARGS... args) {
	bool hasreturned = false;
	R result;
	asyncCall<R>(name, [&result,&hasreturned](const R &r) {
		hasreturned = true;
		result = r;
	}, std::forward<ARGS>(args)...);
	
	// Loop the network
	int limit = 10;
	while (limit > 0 && !hasreturned) {
		limit--;
		ftl::net::wait();
	}
	
	return result;
}

template <typename T, typename... ARGS>
void Socket::asyncCall(
		const std::string &name,
		std::function<void(const T&)> cb,
		ARGS... args) {
	auto args_obj = std::make_tuple(args...);
	auto rpcid = rpcid__++;
	auto call_obj = std::make_tuple(0,rpcid,name,args_obj);
	
	LOG(INFO) << "RPC " << name << "() -> " << uri_;
	
	std::stringstream buf;
	msgpack::pack(buf, call_obj);
	
	// Register the CB
	callbacks_[rpcid] = std::make_unique<caller<T>>(cb);
	
	send(FTL_PROTOCOL_RPC, buf.str());
}

};
};

#endif // _FTL_NET_SOCKET_HPP_
