#ifndef _FTL_NET_PEER_HPP_
#define _FTL_NET_PEER_HPP_

#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <glog/logging.h>
#include <ftl/net/protocol.hpp>
#include <ftl/uri.hpp>
#include <ftl/uuid.hpp>

#ifndef WIN32
#define INVALID_SOCKET -1
#include <netinet/in.h>
#else
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
extern int setDescriptors();

namespace ftl {
namespace net {

class Universe;

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
class Peer {
	public:
	friend class Universe;
	
	enum Status {
		kInvalid, kConnecting, kConnected, kDisconnected, kReconnecting
	};

	public:
	explicit Peer(const char *uri);
	explicit Peer(int s);
	~Peer();
	
	/**
	 * Close the peer if open. Setting retry parameter to true will initiate
	 * backoff retry attempts.
	 */
	void close(bool retry=false);

	bool isConnected() const {
		return sock_ != INVALID_SOCKET && status_ == kConnected;
	};
	
	bool isValid() const {
		return status_ != kInvalid && sock_ != INVALID_SOCKET;
	};
	
	Status status() const { return status_; }
	
	uint32_t getFTLVersion() const { return version_; }
	uint8_t getFTLMajor() const { return version_ >> 16; }
	uint8_t getFTLMinor() const { return (version_ >> 8) & 0xFF; }
	uint8_t getFTLPatch() const { return version_ & 0xFF; }
	
	/**
	 * Get the sockets protocol, address and port as a url string. This will be
	 * the same as the initial connection string on the client.
	 */
	std::string getURI() const { return uri_; };
	
	/**
	 * Get the UUID for this peer.
	 */
	const ftl::UUID &id() const { return peerid_; };
	
	/**
	 * Get the peer id as a string.
	 */
	std::string to_string() const { return peerid_.to_string(); };
			
	/**
	 * Non-blocking Remote Procedure Call using a callback function.
	 */
	template <typename T, typename... ARGS>
	void asyncCall(const std::string &name,
			std::function<void(const T&)> cb,
			ARGS... args);
	
	/**
	 * Blocking Remote Procedure Call using a string name.
	 */
	template <typename R, typename... ARGS>
	R call(const std::string &name, ARGS... args);
	
	/**
	 * Non-blocking send using RPC function, but with no return value.
	 */
	template <typename... ARGS>
	int send(const std::string &name, ARGS... args);
	
	/**
	 * Bind a function to an RPC call name.
	 */
	template <typename F>
	void bind(const std::string &name, F func);

	//void onError(std::function<void(Socket&, int err, const char *msg)> &f) {}
	void onConnect(std::function<void()> &f);
	void onDisconnect(std::function<void()> &f) {}
	
	public:
	static const int kMaxMessage = 10*1024*1024;  // 10Mb currently
	
	protected:
	bool data();			// Process one message from socket
	void socketError();		// Process one error from socket
	void error(int e);
	
	/**
	 * Get the internal OS dependent socket.
	 * TODO(nick) Work out if this should be private.
	 */
	int _socket() const { return sock_; };
	
	/**
	 * Internal handlers for specific event types. This should be private but
	 * is currently here for testing purposes.
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
	Status status_;
	int sock_;
	ftl::URI::scheme_t scheme_;
	uint32_t version_;
	
	// Receive buffers
	msgpack::unpacker recv_buf_;
	
	// Send buffers
	msgpack::vrefbuffer send_buf_;
	
	std::string uri_;
	ftl::UUID peerid_;
	
	ftl::net::Dispatcher disp_;
	std::vector<std::function<void()>> open_handlers_;
	//std::vector<std::function<void(const ftl::net::Error &)>> error_handlers_
	std::vector<std::function<void()>> close_handlers_;
	std::map<int, std::unique_ptr<virtual_caller>> callbacks_;
	
	static int rpcid__;
};

// --- Inline Template Implementations -----------------------------------------

template <typename... ARGS>
int Peer::send(const std::string &s, ARGS... args) {
	// Leave a blank entry for websocket header
	if (scheme_ == ftl::URI::SCHEME_WS) send_buf_.append_ref(nullptr,0);
	//msgpack::pack(send_buf_, std::make_tuple(s, std::make_tuple(args...)));
	auto args_obj = std::make_tuple(args...);
	auto call_obj = std::make_tuple(0,s,args_obj);
	msgpack::pack(send_buf_, call_obj);
	return _send();
}

template <typename F>
void Peer::bind(const std::string &name, F func) {
	disp_.bind(name, func,
		typename ftl::internal::func_kind_info<F>::result_kind(),
	    typename ftl::internal::func_kind_info<F>::args_kind());
}

/*template <typename T>
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
}*/

/*template <typename... ARGS>
int Peer::_send(const std::string &t, ARGS... args) {
	//send_vec_.push_back({const_cast<char*>(t.data()),t.size()});
	//header_w_->size += t.size();
	msgpack::pack(send_buf_, t);
	return _send(args...)+t.size();
}

template <typename... ARGS>
int Peer::_send(const ftl::net::array &b, ARGS... args) {
	//send_vec_.push_back({const_cast<char*>(std::get<0>(b)),std::get<1>(b)});
	//header_w_->size += std::get<1>(b);
	msgpack::pack(send_buf_, msgpack::type::raw_ref(std::get<0>(b), std::get<1>(b)));
	return std::get<1>(b)+_send(args...);
}

template <typename T, typename... ARGS>
int Peer::_send(const std::vector<T> &t, ARGS... args) {
	//send_vec_.push_back({const_cast<char*>(t.data()),t.size()});
	//header_w_->size += t.size();
	msgpack::pack(send_buf_, t);
	return _send(args...)+t.size();
}

template <typename... Types, typename... ARGS>
int Peer::_send(const std::tuple<Types...> &t, ARGS... args) {
	//send_vec_.push_back({const_cast<char*>((char*)&t),sizeof(t)});
	//header_w_->size += sizeof(t);
	msgpack::pack(send_buf_, t);
	return sizeof(t)+_send(args...);
}

template <typename T, typename... ARGS,
		ENABLE_IF(std::is_trivial<T>::value && !std::is_pointer<T>::value)>
int Peer::_send(const T &t, ARGS... args) {
	//send_vec_.push_back({const_cast<T*>(&t),sizeof(T)});
	//header_w_->size += sizeof(T);
	msgpack::pack(send_buf_, t);
	return sizeof(T)+_send(args...);
}*/

//template <typename T, typename... ARGS>
template <typename R, typename... ARGS>
R Peer::call(const std::string &name, ARGS... args) {
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
		// TODO REPLACE ftl::net::wait();
	}
	
	return result;
}

template <typename T, typename... ARGS>
void Peer::asyncCall(
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
	
	send("__rpc__", buf.str());
}

};
};

#endif // _FTL_NET_SOCKET_HPP_
