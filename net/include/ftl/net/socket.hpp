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

namespace ftl {
namespace net {

class Socket {
	public:
	Socket(const char *uri);
	Socket(int s);
	~Socket();
	
	int close();

	int send(uint32_t service, const std::string &data);
	int send(uint32_t service, std::stringstream &data) {
		return send(service, data.str()); };
	int send(uint32_t service, void *data, int length);
	
	int send2(uint32_t service, const std::string &data1,
			const std::string &data2);

	//friend bool ftl::net::run(bool);

	int _socket() const { return sock_; };

	bool isConnected() const { return sock_ != INVALID_SOCKET && connected_; };
	bool isValid() const { return valid_ && sock_ != INVALID_SOCKET; };
	std::string getURI() const { return uri_; };
	
	/**
	 * Bind a function to a RPC call name.
	 */
	template <typename F>
	void bind(const std::string &name, F func) {
		disp_.bind(name, func,
			typename ftl::internal::func_kind_info<F>::result_kind(),
		    typename ftl::internal::func_kind_info<F>::args_kind());
	}
	
	/**
	 * Bind a function to a raw message type.
	 */
	void bind(uint32_t service, std::function<void(Socket&,
			const std::string&)> func);
	
	/**
	 * Remote Procedure Call.
	 */
	template <typename T, typename... ARGS>
	T call(const std::string &name, ARGS... args) {
		bool hasreturned = false;
		T result;
		async_call(name, [&result,&hasreturned](msgpack::object &r) {
			hasreturned = true;
			result = r.as<T>();
		}, std::forward<ARGS>(args)...);
		
		// Loop the network
		int limit = 10;
		while (limit > 0 && !hasreturned) {
			limit--;
			ftl::net::wait();
		}
		
		return result;
	}
	
	template <typename... ARGS>
	void async_call(
			const std::string &name,
			std::function<void(msgpack::object&)> cb,
			ARGS... args) {
		auto args_obj = std::make_tuple(args...);
		auto rpcid = rpcid__++;
		auto call_obj = std::make_tuple(0,rpcid,name,args_obj);
		
		LOG(INFO) << "RPC " << name << "() -> " << uri_;
		
		std::stringstream buf;
		msgpack::pack(buf, call_obj);
		
		// Register the CB
		callbacks_[rpcid] = cb;
		
		send(FTL_PROTOCOL_RPC, buf.str());
	}
	
	/**
	 * Internal handlers for specific event types.
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

	private:
	std::string uri_;
	int sock_;
	size_t pos_;
	char *buffer_;
	std::map<uint32_t,std::function<void(Socket&,const std::string&)>> handlers_;
	std::vector<std::function<void(Socket&)>> connect_handlers_;
	bool valid_;
	bool connected_;
	std::map<int, std::function<void(msgpack::object&)>> callbacks_;
	ftl::net::Dispatcher disp_;
	uint32_t version_;
	std::string peerid_;
	
	void _connected();
	void _updateURI();
	
	static int rpcid__;

	static const int MAX_MESSAGE = 10*1024*1024; // 10Mb currently
	static const int BUFFER_SIZE = MAX_MESSAGE + 16;
};

};
};

#endif // _FTL_NET_SOCKET_HPP_
