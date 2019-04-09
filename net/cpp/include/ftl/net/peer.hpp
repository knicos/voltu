#ifndef _FTL_NET_PEER_HPP_
#define _FTL_NET_PEER_HPP_

#ifndef WIN32
#define INVALID_SOCKET -1
#include <netinet/in.h>
#else
#include <winsock2.h>
#endif

#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <glog/logging.h>
#include <ftl/net/protocol.hpp>
#include <ftl/net/dispatcher.hpp>
#include <ftl/uri.hpp>
#include <ftl/uuid.hpp>

#include <iostream>
#include <sstream>
#include <tuple>
#include <vector>
#include <type_traits>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>

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

//typedef std::tuple<const char*,size_t> array;
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
	friend class Dispatcher;
	
	enum Status {
		kInvalid, kConnecting, kConnected, kDisconnected, kReconnecting
	};

	public:
	explicit Peer(const char *uri, ftl::net::Dispatcher *d=nullptr);
	explicit Peer(int s, ftl::net::Dispatcher *d=nullptr);
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
	 * Bind a function to an RPC call name. Note: if an overriding dispatcher
	 * is used then these bindings will propagate to all peers sharing that
	 * dispatcher.
	 */
	template <typename F>
	void bind(const std::string &name, F func);

	//void onError(std::function<void(Socket&, int err, const char *msg)> &f) {}
	void onConnect(std::function<void()> &f);
	void onDisconnect(std::function<void()> &f) {}
	
	bool isWaiting() const { return is_waiting_; }
	
	public:
	static const int kMaxMessage = 10*1024*1024;  // 10Mb currently
	
	protected:
	void data();			// Process one message from socket
	void socketError();		// Process one error from socket
	void error(int e);
	
	bool _data();
	
	void _dispatchResponse(uint32_t id, msgpack::object &obj);
	void _sendResponse(uint32_t id, const msgpack::object &obj);
	
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
	
	int _send();
	
	/*template <typename... ARGS>
	int _send(const std::string &t, ARGS... args);
	
	template <typename... ARGS>
	int _send(const array &b, ARGS... args);
	
	template <typename T, typename... ARGS>
	int _send(const std::vector<T> &t, ARGS... args);
	
	template <typename... Types, typename... ARGS>
	int _send(const std::tuple<Types...> &t, ARGS... args);
	
	template <typename T, typename... ARGS,
			ENABLE_IF(std::is_trivial<T>::value && !std::is_pointer<T>::value)>
	int _send(const T &t, ARGS... args);*/

	private: // Data
	Status status_;
	int sock_;
	ftl::URI::scheme_t scheme_;
	uint32_t version_;
	bool destroy_disp_;
	
	// Receive buffers
	bool is_waiting_;
	msgpack::unpacker recv_buf_;
	std::mutex recv_mtx_;
	
	// Send buffers
	msgpack::vrefbuffer send_buf_;
	
	std::string uri_;
	ftl::UUID peerid_;
	
	ftl::net::Dispatcher *disp_;
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
	auto args_obj = std::make_tuple(args...);
	auto call_obj = std::make_tuple(0,s,args_obj);
	msgpack::pack(send_buf_, call_obj);
	return _send();
}

template <typename F>
void Peer::bind(const std::string &name, F func) {
	disp_->bind(name, func,
		typename ftl::internal::func_kind_info<F>::result_kind(),
	    typename ftl::internal::func_kind_info<F>::args_kind());
}

template <typename R, typename... ARGS>
R Peer::call(const std::string &name, ARGS... args) {
	bool hasreturned = false;
	std::mutex m;
	std::condition_variable cv;
	
	R result;
	asyncCall<R>(name, [&](const R &r) {
		std::unique_lock<std::mutex> lk(m);
		hasreturned = true;
		result = r;
		lk.unlock();
		cv.notify_one();
	}, std::forward<ARGS>(args)...);
	
	{  // Block thread until async callback notifies us
		std::unique_lock<std::mutex> lk(m);
		cv.wait_for(lk, std::chrono::seconds(1), [&hasreturned]{return hasreturned;});
	}
	
	if (!hasreturned) {
		// TODO(nick) remove callback
		throw 1;
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
	
	msgpack::pack(send_buf_, call_obj);
	
	// Register the CB
	callbacks_[rpcid] = std::make_unique<caller<T>>(cb);
	
	_send();
}

};
};

#endif // _FTL_NET_SOCKET_HPP_
