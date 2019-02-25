#include "catch.hpp"
#include <ftl/net/protocol.hpp>

using ftl::net::Protocol;

// --- Mock --------------------------------------------------------------------

#define _FTL_NET_SOCKET_HPP_ // Prevent include

namespace ftl {
namespace net {
class Socket {
	public:
	std::string getURI() { return "mock://"; }
	int send(int msg, const std::string &d) { return 0; }
};
};
};

using ftl::net::Socket;

class MockProtocol : public Protocol {
	public:
	MockProtocol() : Protocol(33) {}
	void mock_dispatchRPC(Socket &s, const std::string &d) { dispatchRPC(s,d); }
	void mock_dispatchReturn(Socket &s, const std::string &d) { dispatchReturn(s,d); }
	void mock_dispatchRaw(uint32_t msg, Socket &s) { dispatchRaw(msg,s); }
};

// --- Support -----------------------------------------------------------------

// --- Files to test -----------------------------------------------------------

#include "../src/protocol.cpp"
#include "../src/dispatcher.cpp"

// --- Tests -------------------------------------------------------------------

TEST_CASE("Protocol::bind(int,...)", "[proto]") {
	MockProtocol p;
	Socket s;
	
	SECTION("a valid bind and dispatch") {
		bool msg = false;

		p.bind(5, [&](uint32_t m, Socket &s) {
			msg = true;	
		});
		
		p.mock_dispatchRaw(5, s);
		REQUIRE(msg);
	}
	
	SECTION("an invalid dispatch") {
		bool msg = false;

		p.bind(5, [&](uint32_t m, Socket &s) {
			msg = true;	
		});
		
		p.mock_dispatchRaw(6, s);
		REQUIRE( !msg );
		// TODO How is failure reported?
	}
}

TEST_CASE("Protocol::bind(string,...)", "[proto]") {
	MockProtocol p;
	Socket s;
	
	SECTION("no argument bind with valid dispatch") {
		bool called = false;
		
		p.bind("test1", [&]() {
			called = true;
		});
		
		auto args_obj = std::make_tuple();
		auto call_obj = std::make_tuple(0,0,"test1",args_obj);
		std::stringstream buf;
		msgpack::pack(buf, call_obj);
		
		p.mock_dispatchRPC(s, buf.str());
		REQUIRE( called );
	}
	
	SECTION("multiple bindings") {
		bool called1 = false;
		bool called2 = false;
		
		p.bind("test1", [&]() {
			called1 = true;
		});
		p.bind("test2", [&]() {
			called2 = true;
		});
		
		auto args_obj = std::make_tuple();
		auto call_obj = std::make_tuple(0,0,"test2",args_obj);
		std::stringstream buf;
		msgpack::pack(buf, call_obj);
		
		p.mock_dispatchRPC(s, buf.str());
		REQUIRE( !called1 );
		REQUIRE( called2 );
	}
	
	SECTION("one argument bind with valid dispatch") {
		bool called = false;
		
		p.bind("test1", [&](int a) {
			called = true;
			REQUIRE( a == 5 );
		});
		
		auto args_obj = std::make_tuple(5);
		auto call_obj = std::make_tuple(0,0,"test1",args_obj);
		std::stringstream buf;
		msgpack::pack(buf, call_obj);
		
		p.mock_dispatchRPC(s, buf.str());
		REQUIRE( called );
	}
	
	SECTION("two argument bind fake dispatch") {
		bool called = false;
		
		p.bind("test1", [&](int a, float b) {
			called = true;
			REQUIRE( a == 5 );
			REQUIRE( b == 5.4f ); // Danger
		});
		
		auto args_obj = std::make_tuple(5, 5.4f);
		auto call_obj = std::make_tuple(0,0,"test1",args_obj);
		std::stringstream buf;
		msgpack::pack(buf, call_obj);
		
		p.mock_dispatchRPC(s, buf.str());
		REQUIRE( called );
	}
	
	SECTION("non-void bind no arguments") {
		bool called = false;
		
		p.bind("test1", [&]() -> int {
			called = true;
			return 55;
		});
		
		auto args_obj = std::make_tuple();
		auto call_obj = std::make_tuple(0,0,"test1",args_obj);
		std::stringstream buf;
		msgpack::pack(buf, call_obj);
		
		p.mock_dispatchRPC(s, buf.str());
		REQUIRE( called );
		
		// TODO Require that a writev occurred with result value
	}
}

