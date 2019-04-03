#include "catch.hpp"
#include <ftl/net/dispatcher.hpp>
#include <ftl/net/protocol.hpp>
#include <ftl/net/socket.hpp>
#include <memory>
#include <iostream>

#ifndef WIN32
#include <sys/select.h>
#else
typedef int ssize_t;
#endif

using ftl::net::Dispatcher;
using ftl::net::Protocol;
using ftl::net::Socket;

// --- Mock --------------------------------------------------------------------

static std::vector<std::string> last_send;
static int last_pos = 0;

using ftl::net::Socket;

class MockSocket : public Socket {
	public:	
	MockSocket() : Socket(0) {}
	void mock_dispatchRPC(const std::string &d) { protocol()->dispatchRPC(*this,d); }
	
	void mock_data() { data(); }

};

extern int select(int nfds, fd_set *readfds, fd_set *writefds,
                  fd_set *exceptfds, struct timeval *timeout) {
   // std::cout << "SELECT CALLED" << std::endl;
    FD_ZERO(exceptfds);
    FD_ZERO(readfds);
    if ((size_t)last_pos < last_send.size()) FD_SET(0, readfds);
	return 1;      
}

extern ssize_t recv(int sd, void *buf, size_t n, int f) {	
	if ((size_t)last_pos >= last_send.size()) return 0;
	//std::cout << "Recv called : " << last_send[last_pos].size() << std::endl;
	int l = last_send[last_pos].size();
	if (l == 0) return 0;
	std::memcpy(buf, last_send[last_pos].c_str(), l);
	//last_send.erase(last_send.begin());
	last_pos++;
	return l;
}

extern ssize_t writev(int sd, const struct iovec *v, int cnt) {
	size_t len = 0; //v[0].iov_len+v[1].iov_len;
	char buf[1000];
	char *bufp = &buf[0];
	
	for (auto i=0; i<cnt; i++) {
		std::memcpy(bufp,v[i].iov_base,v[i].iov_len);
		len += v[i].iov_len;
		bufp += v[i].iov_len;
	}
	
	//std::cout << "WRITEV " << len << std::endl;
	
	//if (last_send.size() != 0) std::cout << "ERROR MISSED MESSAGE " << last_send.size() << std::endl;
	last_send.push_back(std::string(&buf[0], len));
	return len;
}

extern std::vector<std::shared_ptr<ftl::net::Socket>> sockets;

// --- Support -----------------------------------------------------------------

Dispatcher::response_t get_response() {
	auto h = (ftl::net::Header*)last_send[last_pos].data();
	const char *data = last_send[last_pos].data() + sizeof(ftl::net::Header);
	auto unpacked = msgpack::unpack(data, h->size-4);
	Dispatcher::response_t the_result;
	unpacked.get().convert(the_result);
	return the_result;
}

// --- Tests -------------------------------------------------------------------

#include <ftl/net/p2p.hpp>

using ftl::net::P2P;

SCENARIO("p2p::bind_find_one()", "[find_one]") {
	class Mock_p2p : public P2P {
		public:
		Mock_p2p() : P2P("mock://") {
			bind_find_one("test", &Mock_p2p::test);
		}
		
		std::optional<int> test(int a) {
			if (a == 2) return 44;
			else return {};
		}
	};
	
	Mock_p2p p;
	std::shared_ptr<MockSocket> s = std::shared_ptr<MockSocket>(new MockSocket());
	s->setProtocol(&p);
	sockets.push_back(s);
	p.addPeer(s);
	
	GIVEN("a query that expects a valid result") {
		// Create a mock RPC message with expected result
		ftl::UUID req;
		int ttl = 10;
		auto args_obj = std::make_tuple(req, ttl, 2);
		auto call_obj = std::make_tuple(0,0,"test",args_obj);
		std::stringstream buf;
		msgpack::pack(buf, call_obj);
		
		s->mock_dispatchRPC(buf.str());
		
		// Make sure we get a response
		auto [kind,id,err,res] = get_response();
		REQUIRE( *(res.as<std::optional<int>>()) == 44 );
		REQUIRE( kind == 1 );
		REQUIRE( id == 0 );
		REQUIRE( err.type == 0 );
	}
	
	GIVEN("a query that expects no result") {
		// Create a mock RPC message with expected result
		ftl::UUID req;
		int ttl = 10;
		auto args_obj = std::make_tuple(req, ttl, 3);
		auto call_obj = std::make_tuple(0,0,"test",args_obj);
		std::stringstream buf;
		msgpack::pack(buf, call_obj);
		s->mock_dispatchRPC(buf.str());
		
		// Make sure we get a response
		auto [kind,id,err,res] = get_response();
		REQUIRE( !res.as<std::optional<int>>() );
		REQUIRE( kind == 1 );
		REQUIRE( id == 0 );
		REQUIRE( err.type == 0 );
	}
	
	last_pos = 0;
	last_send.clear();
	ftl::net::stop();
}

SCENARIO("p2p::bind_find_all()", "[find_one]") {
	class Mock_p2p : public P2P {
		public:
		Mock_p2p() : P2P("mock://") {
			bind_find_all("test", &Mock_p2p::test);
		}
		
		std::optional<int> test(int a) {
			if (a == 2) return 44;
			else return {};
		}
	};
	
	Mock_p2p p,p2;
	std::shared_ptr<MockSocket> s = std::shared_ptr<MockSocket>(new MockSocket());
	s->setProtocol(&p);
	sockets.push_back(s);
	p.addPeer(s);
	
	GIVEN("a query that expects valid results") {
		// Create a mock RPC message with expected result
		ftl::UUID req;
		int ttl = 10;
		auto args_obj = std::make_tuple(req, ttl, 2);
		auto call_obj = std::make_tuple(0,0,"test",args_obj);
		std::stringstream buf;
		msgpack::pack(buf, call_obj);
		
		s->mock_dispatchRPC(buf.str());
		
		// Make sure we get a response
		auto [kind,id,err,res] = get_response();
		auto vec = res.as<std::vector<int>>();
		REQUIRE( vec.size() == 1 );
		REQUIRE( vec[0] == 44 );
		REQUIRE( kind == 1 );
		REQUIRE( id == 0 );
		REQUIRE( err.type == 0 );
	}
	
	GIVEN("a query that expects no result") {
		// Create a mock RPC message with expected result
		ftl::UUID req;
		int ttl = 10;
		auto args_obj = std::make_tuple(req, ttl, 3);
		auto call_obj = std::make_tuple(0,0,"test",args_obj);
		std::stringstream buf;
		msgpack::pack(buf, call_obj);
		s->mock_dispatchRPC(buf.str());
		
		// Make sure we get a response
		auto [kind,id,err,res] = get_response();
		REQUIRE( res.as<std::vector<int>>().size() == 0 );
		REQUIRE( kind == 1 );
		REQUIRE( id == 0 );
		REQUIRE( err.type == 0 );
	}
	
	last_pos = 0;
	last_send.clear();
	ftl::net::stop();
}

SCENARIO("p2p::find_one()", "[find_one]") {
	class Mock_p2p : public P2P {
		public:
		Mock_p2p() : P2P("mock://") {
			bind_find_one("test", &Mock_p2p::test);
		}
		
		std::optional<int> test(int a) {
			if (a == 2) return 44;
			else return {};
		}
	};
	
	Mock_p2p p;
	std::shared_ptr<MockSocket> s = std::shared_ptr<MockSocket>(new MockSocket());
	sockets.push_back(s);
	s->setProtocol(&p);
	p.addPeer(s);
	
	GIVEN("a query that expects a valid result") {
		auto res = p.find_one<int>("test", 2);		
		REQUIRE( res.has_value() );
		REQUIRE( *res == 44 );
	}
	
	GIVEN("a query that expects no result") {
		auto res = p.find_one<int>("test", 3);		
		REQUIRE( !res.has_value() );
	}
	
	last_pos = 0;
	last_send.clear();
	ftl::net::stop();
}

SCENARIO("p2p::find_all()", "[find_one]") {
	class Mock_p2p : public P2P {
		public:
		Mock_p2p() : P2P("mock://") {
			bind_find_all("test", &Mock_p2p::test);
		}
		
		std::optional<int> test(int a) {
			if (a == 2) return 44;
			else return {};
		}
	};
	
	Mock_p2p p;
	std::shared_ptr<MockSocket> s = std::shared_ptr<MockSocket>(new MockSocket());
	sockets.push_back(s);
	s->setProtocol(&p);
	p.addPeer(s);
	
	GIVEN("a query that expects a valid result") {
		auto res = p.find_all<int>("test", 2);		
		REQUIRE( res.size() == 1 );
		REQUIRE( res[0] == 44 );
	}
	
	GIVEN("a query that expects no result") {
		auto res = p.find_all<int>("test", 3);		
		REQUIRE( res.size() == 0 );
	}
	
	last_pos = 0;
	last_send.clear();
	ftl::net::stop();
}

