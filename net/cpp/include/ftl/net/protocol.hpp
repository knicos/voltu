#ifndef _FTL_NET_PROTOCOL_HPP_
#define _FTL_NET_PROTOCOL_HPP_

#include <ftl/net/func_traits.hpp>
#include <ftl/net/dispatcher.hpp>
#include <map>
#include <string>

#define FTL_PROTOCOL_HS1		0x0001		// Handshake step 1
#define FTL_PROTOCOL_HS2		0x0002		// Handshake step 2

#define FTL_PROTOCOL_RPC		0x0100
#define FTL_PROTOCOL_RPCRETURN	0x0101

#define FTL_PROTOCOL_FREE		0x1000		// Custom protocols above this

namespace ftl {
namespace net {

class Reader;
class Socket;

#pragma pack(push,1)

struct Header {
	uint32_t size;
	uint32_t service;
};

struct Handshake {
	uint64_t magic;
	uint32_t name_size;
	uint32_t proto_size;
};

#pragma pack(pop)

static const uint64_t MAGIC = 0x1099340053640912;

/**
 * Each instance of this Protocol class represents a specific protocol. A
 * protocol is a set of RPC bindings and raw message handlers. A protocol is
 * identified, selected and validated using an automatically generated hash of
 * all its supported bindings. The choice of protocol for a socket is made
 * during the initial connection handshake.
 */
class Protocol {
	public:
	friend class Socket;
	
	public:
	explicit Protocol(const std::string &id);
	~Protocol();
	
	/**
	 * Bind a function to an RPC call name.
	 */
	template <typename F>
	void bind(const std::string &name, F func);
	
	/**
	 * Bind a function to a raw message type.
	 */
	void bind(int service, std::function<void(uint32_t,Socket&)> func);
			
	// broadcast?
	
	const std::string &id() const { return id_; }
	
	static Protocol *find(const std::string &id);
			
	//protected:
	void dispatchRPC(Socket &, const std::string &d);
	void dispatchReturn(Socket &, const std::string &d);
	void dispatchRaw(uint32_t service, Socket &);
	
	void addSocket(std::shared_ptr<Socket> s);
	void removeSocket(const Socket &s);
	
	private:
	ftl::net::Dispatcher disp_;
	std::map<uint32_t,std::function<void(uint32_t,Socket&)>> handlers_;
	std::string id_;
	
	static std::map<std::string,Protocol*> protocols__;
};

// --- Template Implementations ------------------------------------------------

template <typename F>
void Protocol::bind(const std::string &name, F func) {
	disp_.bind(name, func,
		typename ftl::internal::func_kind_info<F>::result_kind(),
	    typename ftl::internal::func_kind_info<F>::args_kind());
}

};
};

#endif // _FTL_NET_PROTOCOL_HPP_
