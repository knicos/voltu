/**
 * @file peer.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#ifndef _FTL_NET_PEER_HPP_
#define _FTL_NET_PEER_HPP_

#ifndef NOMINMAX
#define NOMINMAX
#endif


#include <ftl/utility/msgpack.hpp>
#include <ftl/net/common_fwd.hpp>
#include <ftl/exception.hpp>

#include <ftl/net/protocol.hpp>
#include <ftl/net/dispatcher.hpp>
#include <ftl/uri.hpp>
#include <ftl/uuid.hpp>
#include <ftl/threads.hpp>
#include <ftl/timer.hpp>

#include <iostream>
#include <sstream>
#include <tuple>
#include <vector>
#include <type_traits>
#include <thread>
#include <condition_variable>
#include <chrono>

# define ENABLE_IF(...) \
  typename std::enable_if<(__VA_ARGS__), bool>::type = true

extern bool _run(bool blocking, bool nodelay);
extern int setDescriptors();

namespace ftl {
namespace net {

extern ftl::UUID this_peer;

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
	explicit Peer(const char *uri, ftl::net::Universe *, ftl::net::Dispatcher *d=nullptr);
	explicit Peer(SOCKET s, ftl::net::Universe *, ftl::net::Dispatcher *d=nullptr);
	~Peer();
	
	/**
	 * Close the peer if open. Setting retry parameter to true will initiate
	 * backoff retry attempts. This is used to deliberately close a connection
	 * and not for error conditions where different close semantics apply.
	 * 
	 * @param retry Should reconnection be attempted?
	 */
	void close(bool retry=false);

	bool isConnected() const {
		return sock_ != INVALID_SOCKET && status_ == kConnected;
	};

	/**
	 * Block until the connection and handshake has completed. You should use
	 * onConnect callbacks instead of blocking, mostly this is intended for
	 * the unit tests to keep them synchronous.
	 * 
	 * @return True if all connections were successful, false if timeout or error.
	 */
	bool waitConnection();

	/**
	 * Make a reconnect attempt. Called internally by Universe object.
	 */
	bool reconnect();

	inline bool isOutgoing() const { return outgoing_; }
	
	/**
	 * Test if the connection is valid. This returns true in all conditions
	 * except where the socket has been disconnected permenantly or was never
	 * able to connect, perhaps due to an invalid address.
	 */
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
	 * 
	 * @param name RPC Function name.
	 * @param cb Completion callback.
	 * @param args A variable number of arguments for RPC function.
	 * 
	 * @return A call id for use with cancelCall() if needed.
	 */
	template <typename T, typename... ARGS>
	int asyncCall(const std::string &name,
			std::function<void(const T&)> cb,
			ARGS... args);

	/**
	 * Used to terminate an async call if the response is not required.
	 * 
	 * @param id The ID returned by the original asyncCall request.
	 */
	void cancelCall(int id);
	
	/**
	 * Blocking Remote Procedure Call using a string name.
	 */
	template <typename R, typename... ARGS>
	R call(const std::string &name, ARGS... args);
	
	/**
	 * Non-blocking send using RPC function, but with no return value.
	 * 
	 * @param name RPC Function name
	 * @param args Variable number of arguments for function
	 * 
	 * @return Number of bytes sent or -1 if error
	 */
	template <typename... ARGS>
	int send(const std::string &name, ARGS... args);

	template <typename... ARGS>
	int try_send(const std::string &name, ARGS... args);
	
	/**
	 * Bind a function to an RPC call name. Note: if an overriding dispatcher
	 * is used then these bindings will propagate to all peers sharing that
	 * dispatcher.
	 * 
	 * @param name RPC name to bind to
	 * @param func A function object to act as callback
	 */
	template <typename F>
	void bind(const std::string &name, F func);

	// void onError(std::function<void(Peer &, int err, const char *msg)> &f) {}
	//void onConnect(const std::function<void(Peer &)> &f);
	//void onDisconnect(std::function<void(Peer &)> &f) {}
	
	bool isWaiting() const { return is_waiting_; }

	void rawClose() { _badClose(false); }

	inline void noReconnect() { can_reconnect_ = false; }

	inline unsigned int localID() const { return local_id_; }
	
	public:
	static const int kMaxMessage = 10*1024*1024;  // 10Mb currently
	
	protected:
	void data();			// Process one message from socket
	bool socketError();		// Process one error from socket
	void error(int e);
	
	bool _data();

	void _badClose(bool retry=true);
	
	void _dispatchResponse(uint32_t id, const std::string &name, msgpack::object &obj);
	void _sendResponse(uint32_t id, const std::string &name, const msgpack::object &obj);
	
	/**
	 * Get the internal OS dependent socket.
	 * TODO(nick) Work out if this should be private.
	 */
	SOCKET _socket() const { return sock_; };
	
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

	void _waitCall(int id, std::condition_variable &cv, bool &hasreturned, const std::string &name);
	
	template<typename... ARGS>
	void _trigger(const std::vector<std::function<void(Peer &, ARGS...)>> &hs, ARGS... args) {
		for (auto h : hs) {
			h(*this, args...);
		}
	}

	private:
	Status status_;					// Connected, errored, reconnecting...
	SOCKET sock_;					// Raw OS socket
	ftl::URI::scheme_t scheme_;		// TCP / WS / UDP...
	uint32_t version_;				// Received protocol version in handshake
	bool can_reconnect_;			// Client connections can retry
	ftl::net::Universe *universe_;	// Origin net universe
	
	// Receive buffers
	volatile bool is_waiting_;
	msgpack::unpacker recv_buf_;
	RECURSIVE_MUTEX recv_mtx_;
	bool ws_read_header_;
	
	// Send buffers
	msgpack::vrefbuffer send_buf_;
	RECURSIVE_MUTEX send_mtx_;

	RECURSIVE_MUTEX cb_mtx_;
	
	std::string uri_;				// Original connection URI, or assumed URI
	ftl::UUID peerid_;				// Received in handshake or allocated
	bool outgoing_;
	unsigned int local_id_;
	
	ftl::net::Dispatcher *disp_;	// For RPC call dispatch
	//std::vector<std::function<void(Peer &)>> open_handlers_;
	//std::vector<std::function<void(const ftl::net::Error &)>> error_handlers_
	//std::vector<std::function<void(Peer &)>> close_handlers_;
	std::map<int, std::unique_ptr<virtual_caller>> callbacks_;
	
	static std::atomic_int rpcid__;				// Return ID for RPC calls
	static std::atomic_int local_peer_ids__;
};

// --- Inline Template Implementations -----------------------------------------

template <typename... ARGS>
int Peer::send(const std::string &s, ARGS... args) {
	UNIQUE_LOCK(send_mtx_, lk);
	// Leave a blank entry for websocket header
	if (scheme_ == ftl::URI::SCHEME_WS) send_buf_.append_ref(nullptr,0);
	auto args_obj = std::make_tuple(args...);
	auto call_obj = std::make_tuple(0,s,args_obj);
	msgpack::pack(send_buf_, call_obj);
	int rc = _send();
	return rc;
}

/*template <typename... ARGS>
int Peer::try_send(const std::string &s, ARGS... args) {
#ifdef WIN32
	WSAPOLLFD fds;
	fds.fd = sock_;
	fds.events = POLLOUT;
	int rc = WSAPoll(&fds, 1, 0);
#else
	pollfd fds;
	fds.fd = sock_;
	fds.events = POLLOUT;
	int rc = poll(&fds, 1, 0);
#endif
	if (rc == SOCKET_ERROR) return -1;
	if (rc == 0) return 0;

	UNIQUE_LOCK(send_mtx_, lk);
	// Leave a blank entry for websocket header
	if (scheme_ == ftl::URI::SCHEME_WS) send_buf_.append_ref(nullptr,0);
	auto args_obj = std::make_tuple(args...);
	auto call_obj = std::make_tuple(0,s,args_obj);
	msgpack::pack(send_buf_, call_obj);
	rc = _send();
	return (rc < 0) ? -1 : 1;
}*/

template <typename F>
void Peer::bind(const std::string &name, F func) {
	disp_->bind(name, func,
		typename ftl::internal::func_kind_info<F>::result_kind(),
	    typename ftl::internal::func_kind_info<F>::args_kind(),
		typename ftl::internal::func_kind_info<F>::has_peer());
}

template <typename R, typename... ARGS>
R Peer::call(const std::string &name, ARGS... args) {
	bool hasreturned = false;
	//std::mutex m;
	std::condition_variable cv;
	
	R result;
	int id = asyncCall<R>(name, [&](const R &r) {
		result = r;
		hasreturned = true;
		cv.notify_one();
	}, std::forward<ARGS>(args)...);
	
	_waitCall(id, cv, hasreturned, name);
	
	return result;
}

template <typename T, typename... ARGS>
int Peer::asyncCall(
		const std::string &name,
		// cppcheck-suppress *
		std::function<void(const T&)> cb,
		ARGS... args) {
	auto args_obj = std::make_tuple(args...);
	uint32_t rpcid = 0;

	{
		// Could this be the problem????
		UNIQUE_LOCK(cb_mtx_,lk);
		// Register the CB
		rpcid = rpcid__++;
		callbacks_[rpcid] = std::make_unique<caller<T>>(cb);
	}

	//DLOG(INFO) << "RPC " << name << "(" << rpcid << ") -> " << uri_;

	auto call_obj = std::make_tuple(0,rpcid,name,args_obj);
	
	UNIQUE_LOCK(send_mtx_,lk);
	if (scheme_ == ftl::URI::SCHEME_WS) send_buf_.append_ref(nullptr,0);
	msgpack::pack(send_buf_, call_obj);
	_send();
	return rpcid;
}

};
};

#endif // _FTL_NET_SOCKET_HPP_
