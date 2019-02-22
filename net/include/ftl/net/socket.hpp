#ifndef _FTL_NET_SOCKET_HPP_
#define _FTL_NET_SOCKET_HPP_

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
	int send(uint32_t service, std::stringstream &data) { return send(service, data.str()); };
	int send(uint32_t service, void *data, int length);
	
	int send2(uint32_t service, const std::string &data1, const std::string &data2);

	//friend bool ftl::net::run(bool);

	int _socket() { return m_sock; };

	bool isConnected() { return m_sock != INVALID_SOCKET; };
	bool isValid() { return m_valid; };
	
	template <typename F>
	void bind(const std::string &name, F func) {
		//disp_.enforce_unique_name(name);
		disp_.bind(name, func, typename ftl::internal::func_kind_info<F>::result_kind(),
		     typename ftl::internal::func_kind_info<F>::args_kind());
	}
	
	template <typename... ARGS>
	msgpack::object_handle call(const std::string &name, ARGS... args) {
		bool hasreturned = false;
		msgpack::object_handle result;
		async_call(name, [result,hasreturned](msgpack::object_handle r) {
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
	
	template <typename... ARGS>
	void async_call(
			const std::string &name,
			std::function<void(msgpack::object_handle)> cb,
			ARGS... args) {
		auto args_obj = std::make_tuple(args...);
		auto rpcid = rpcid__++;
		auto call_obj = std::make_tuple(0,rpcid,name,args_obj);
		
		std::stringstream buf;
		msgpack::pack(buf, call_obj);
		
		// Register the CB
		callbacks_[rpcid] = cb;
		
		send(FTL_PROTOCOL_RPC, buf.str());
	}
	
	void dispatch(const std::string &b) { disp_.dispatch(b); }

	void onMessage(sockdatahandler_t handler) { m_handler = handler; }
	void onError(sockerrorhandler_t handler) {}
	void onConnect(sockconnecthandler_t handler) {}
	void onDisconnect(sockdisconnecthandler_t handler) {}
	
	bool data();
	void error();

	protected:

	char m_addr[INET6_ADDRSTRLEN];

	private:
	const char *m_uri;
	int m_sock;
	size_t m_pos;
	char *m_buffer;
	sockdatahandler_t m_handler;
	bool m_valid;
	std::map<int, std::function<void(msgpack::object_handle)>> callbacks_;
	ftl::net::Dispatcher disp_;
	
	static int rpcid__;

	static const int MAX_MESSAGE = 10*1024*1024; // 10Mb currently
	static const int BUFFER_SIZE = MAX_MESSAGE + 16;
};

};
};

#endif // _FTL_NET_SOCKET_HPP_
