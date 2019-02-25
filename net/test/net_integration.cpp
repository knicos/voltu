#include "catch.hpp"
#include <ftl/net.hpp>
#include <ftl/net/socket.hpp>
#include <ftl/net/listener.hpp>

#include <memory>

using ftl::net::Socket;
using ftl::net::Protocol;
using std::shared_ptr;

TEST_CASE("Net Integration", "[integrate]") {
	std::string data;
	
	Protocol p(143);
	
	p.bind("add", [](int a, int b) {
		return a + b;
	});
	
	p.bind(100, [&data](uint32_t m, Socket &s) {
		s.read(data);
	});
	
	auto l = ftl::net::listen("tcp://localhost:9000");
	REQUIRE( l->isListening() );
	l->setProtocol(&p);
	
	shared_ptr<Socket> s1;
	l->onConnection([&s1](auto &s) { s1 = s; });
	
	shared_ptr<Socket> s2 = ftl::net::connect("tcp://localhost:9000");
	
	ftl::net::wait(); // TODO, make setProtocol block until handshake complete
	ftl::net::wait();
	REQUIRE( s1 != nullptr );
	REQUIRE( s2 != nullptr );
	
	REQUIRE( s1->isConnected() );
	REQUIRE( s2->isConnected() );
	
	REQUIRE( s1->call<int>("add", 5, 6) == 11 );
	REQUIRE( s2->call<int>("add", 10, 5) == 15);
	
	s1->send(100, "hello world");
	ftl::net::wait();
	// TODO s2->wait(100);
	
	REQUIRE( data == "hello world" );
}

