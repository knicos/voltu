#ifndef _FTL_NET_SOCKET_HPP_
#define _FTL_NET_SOCKET_HPP_

#include <glog/logging.h>
#include <ftl/net.hpp>
#include <ftl/net/handlers.hpp>
#include <ftl/net/dispatcher.hpp>
#include <ftl/net/protocol.hpp>

#ifndef WIN32
#define INVALID_SOCKET -1
#include <netinet/in.h>
#endif

#ifdef WIN32
//#include <windows.h>
#include <winsock.h>
#endif

#include <sstream>
#include <type_traits>

namespace ftl {
namespace net {

struct virtual_caller {
	virtual void operator()(msgpack::object &o)=0;
};

template <typename T>
struct caller : virtual_caller {
	caller(std::function<void(const T&)> f) : f_(f) {};
	void operator()(msgpack::object &o) { T r = o.as<T>(); f_(r); };
	std::function<void(const T&)> f_;
};

/**
 * A single socket connection object, to be constructed using the connect()
 * function and not to be created directly.
 */
class Socket {
	public:
	Socket(const char *uri);
	Socket(int s);
	~Socket();
	
	int close();

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
	 * Bind a function to an RPC call name.
	 */
	template <typename F>
	void bind(const std::string &name, F func);
	
	/**
	 * Bind a function to a raw message type.
	 */
	void bind(uint32_t service, std::function<void(Socket&,
			const std::string&)> func);
			
	/**
	 * Non-blocking Remote Procedure Call using a callback function.
	 */
	template <typename T, typename... ARGS>
	void asyncCall(const std::string &name,
			std::function<void(const T&)> cb,
			ARGS... args);
	// TODO use "call" instead of "acall" causes compiler to loop.
	
	/**
	 * Blocking Remote Procedure Call.
	 */
	template <typename R, typename... ARGS>
	R call(const std::string &name, ARGS... args);
			
	/**
	 * Send data to given service number.
	 */
	int send(uint32_t service, const std::string &data);
	
	/**
	 * Send with two distinct data source. Avoids the need to memcpy them to a
	 * single buffer.
	 */
	int send2(uint32_t service, const std::string &data1,
			const std::string &data2);
	
	/**
	 * Internal handlers for specific event types. This should be private but
	 * is current here for testing purposes.
	 * @{
	 */
	void dispatchRPC(const std::string &d) { disp_.dispatch(d); }
	void dispatchReturn(const std::string &d);
	void handshake1(const std::string &d);
	void handshake2(const std::string &d);
	/** @} */

	void onError(sockerrorhandler_t handler) {}
	void onConnect(std::function<void(Socket&)> f);
	void onDisconnect(sockdisconnecthandler_t handler) {}
	
	bool data();
	void error();
	
	private: // Functions
	void _connected();
	void _updateURI();

	private: // Data
	bool valid_;
	bool connected_;
	uint32_t version_;
	int sock_;
	size_t pos_;
	char *buffer_;
	
	std::string uri_;
	std::string peerid_;
	
	ftl::net::Dispatcher disp_;
	
	std::map<uint32_t,std::function<void(Socket&,const std::string&)>> handlers_;
	std::vector<std::function<void(Socket&)>> connect_handlers_;
	std::map<int, std::unique_ptr<virtual_caller>> callbacks_;
	
	static int rpcid__;

	static const int MAX_MESSAGE = 10*1024*1024; // 10Mb currently
	static const int BUFFER_SIZE = MAX_MESSAGE + 16;
};

// --- Inline Template Implementations -----------------------------------------

template <typename F>
void Socket::bind(const std::string &name, F func) {
	disp_.bind(name, func,
		typename ftl::internal::func_kind_info<F>::result_kind(),
	    typename ftl::internal::func_kind_info<F>::args_kind());
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
