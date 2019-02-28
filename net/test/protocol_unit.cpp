#include "catch.hpp"
#include <ftl/net/protocol.hpp>
#include <ftl/net/dispatcher.hpp>

using ftl::net::Protocol;
using ftl::net::Dispatcher;

// --- Mock --------------------------------------------------------------------

#define _FTL_NET_SOCKET_HPP_ // Prevent include

static std::string last_send;

namespace ftl {
namespace net {
class Socket {
	public:
	std::string getURI() { return "mock://"; }
	int send(int msg, const std::string &d) { last_send = d; return 0; }
};
};
};

using ftl::net::Socket;

class MockProtocol : public Protocol {
	public:
	MockProtocol() : Protocol("ftl://utu.fi") {}
	void mock_dispatchRPC(Socket &s, const std::string &d) { dispatchRPC(s,d); }
	void mock_dispatchReturn(Socket &s, const std::string &d) { dispatchReturn(s,d); }
	void mock_dispatchRaw(uint32_t msg, Socket &s) { dispatchRaw(msg,s); }
};

// --- Support -----------------------------------------------------------------

Dispatcher::response_t get_response() {
	auto unpacked = msgpack::unpack(last_send.data(), last_send.size());
	Dispatcher::response_t the_result;
	unpacked.get().convert(the_result);
	return the_result;
}

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

SCENARIO("Protocol::bind(string,...)", "[proto]") {
	MockProtocol p;
	Socket s;
	
	GIVEN("no arguments and no return") {
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
	
	GIVEN("multiple no argument bindings") {
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
	
	GIVEN("one argument") {
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
	
	GIVEN("two arguments no return") {
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
	
	GIVEN("integer return, no arguments") {
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
		
		auto [kind,id,err,res] = get_response();
		REQUIRE( res.as<int>() == 55 );
		REQUIRE( kind == 1 );
		REQUIRE( id == 0 );
		REQUIRE( err.type == 0 );
	}
	
	GIVEN("integer return and one argument") {
		bool called = false;
		
		p.bind("test1", [&](int a) -> int {
			called = true;
			return a+2;
		});
		
		auto args_obj = std::make_tuple(12);
		auto call_obj = std::make_tuple(0,0,"test1",args_obj);
		std::stringstream buf;
		msgpack::pack(buf, call_obj);
		
		p.mock_dispatchRPC(s, buf.str());
		REQUIRE( called );
		
		auto [kind,id,err,res] = get_response();
		REQUIRE( res.as<int>() == 14 );
		REQUIRE( kind == 1 );
		REQUIRE( id == 0 );
		REQUIRE( err.type == 0 );
	}
	
	GIVEN("an integer exception in bound function") {
		bool called = false;
		
		p.bind("test1", [&](int a) {
			called = true;
			throw -1;
		});
		
		auto args_obj = std::make_tuple(5);
		auto call_obj = std::make_tuple(0,0,"test1",args_obj);
		std::stringstream buf;
		msgpack::pack(buf, call_obj);
		
		p.mock_dispatchRPC(s, buf.str());
		REQUIRE( called );
		
		auto [kind,id,err,res] = get_response();
		REQUIRE( res.type == 0 );
		REQUIRE( kind == 1 );
		REQUIRE( id == 0 );
		REQUIRE( err.as<int>() == -1 );
	}
	
	GIVEN("a custom std::exception in bound function") {
		bool called = false;
		
		struct CustExcept : public std::exception {
			const char *what() const noexcept { return "My exception"; }
		};
		
		p.bind("test1", [&](int a) {
			called = true;
			throw CustExcept();
		});
		
		auto args_obj = std::make_tuple(5);
		auto call_obj = std::make_tuple(0,0,"test1",args_obj);
		std::stringstream buf;
		msgpack::pack(buf, call_obj);
		
		p.mock_dispatchRPC(s, buf.str());
		REQUIRE( called );
		
		auto [kind,id,err,res] = get_response();
		REQUIRE( res.type == 0 );
		REQUIRE( kind == 1 );
		REQUIRE( id == 0 );
		REQUIRE( err.type == 5 );
		REQUIRE( err.as<std::string>() == "My exception" );
	}
}

