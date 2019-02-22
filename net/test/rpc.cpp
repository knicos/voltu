#include "catch.hpp"
#include <ftl/net/socket.hpp>
#include <iostream>

TEST_CASE("Socket::bind()", "[rpc]") {
	SECTION("no argument bind") {
		auto s = new ftl::net::Socket(0);
		bool called = false;
		
		s->bind("test1", [&]() {
			called = true;
		});
		
		auto args_obj = std::make_tuple();
		auto call_obj = std::make_tuple(0,0,"test1",args_obj);
		std::stringstream buf;
		msgpack::pack(buf, call_obj);
		
		s->dispatch(buf.str());
		REQUIRE( called );
	}
	
	SECTION("one argument bind") {
		auto s = new ftl::net::Socket(0);
		bool called = false;
		
		s->bind("test1", [&](int a) {
			called = true;
			REQUIRE( a == 5 );
		});
		
		auto args_obj = std::make_tuple(5);
		auto call_obj = std::make_tuple(0,0,"test1",args_obj);
		std::stringstream buf;
		msgpack::pack(buf, call_obj);
		
		s->dispatch(buf.str());
		REQUIRE( called );
	}
	
	SECTION("two argument bind") {
		auto s = new ftl::net::Socket(0);
		bool called = false;
		
		s->bind("test1", [&](int a, float b) {
			called = true;
			REQUIRE( a == 5 );
			REQUIRE( b == 5.4f ); // Danger
		});
		
		auto args_obj = std::make_tuple(5, 5.4f);
		auto call_obj = std::make_tuple(0,0,"test1",args_obj);
		std::stringstream buf;
		msgpack::pack(buf, call_obj);
		
		s->dispatch(buf.str());
		REQUIRE( called );
	}
}

