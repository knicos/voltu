#include "catch.hpp"
#include <ftl/net.hpp>

#include <thread>
#include <chrono>

using ftl::net::Universe;

// --- Support -----------------------------------------------------------------

// --- Tests -------------------------------------------------------------------

TEST_CASE("Universe::connect()", "[net]") {
	Universe a("ftl://utu.fi");
	Universe b("ftl://utu.fi");
	
	a.listen("tcp://localhost:7077");

	SECTION("valid tcp connection using ipv4") {
		REQUIRE( b.connect("tcp://127.0.0.1:7077") );
		
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
		
		REQUIRE( a.numberOfPeers() == 1 );
		REQUIRE( b.numberOfPeers() == 1 );
	}

	SECTION("valid tcp connection using hostname") {
		REQUIRE( b.connect("tcp://localhost:7077") );
		
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
		
		REQUIRE( a.numberOfPeers() == 1 );
		REQUIRE( b.numberOfPeers() == 1 );
	}

	SECTION("invalid protocol") {
		REQUIRE( !b.connect("http://127.0.0.1:7077") );
		
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
		
		REQUIRE( a.numberOfPeers() == 0 );
		REQUIRE( b.numberOfPeers() == 0 );
	}

	/*SECTION("empty uri") {
		sock = ftl::net::connect("");
		REQUIRE(!sock->isValid());
	}

	SECTION("null uri") {
		sock = ftl::net::connect(NULL);
		REQUIRE(!sock->isValid());
	}*/

	// Disabled due to long timeout
	/*SECTION("incorrect ipv4 address") {
		sock = ftl::net::raw::connect("tcp://192.0.1.1:7077");
		REQUIRE(sock != NULL);
		REQUIRE(sock->isConnected() == false);
		sock = NULL;
	}*/

	// Removed as too slow
	/*SECTION("incorrect dns address") {
		sock = ftl::net::connect("tcp://xryyrrgrtgddgr.com:7077");
		REQUIRE(!sock->isValid());
	}*/
	
	//fin_server();
}

TEST_CASE("Universe::broadcast()", "[net]") {
	Universe a("ftl://utu.fi");
	Universe b("ftl://utu.fi");
	
	a.listen("tcp://localhost:7077");
	
	SECTION("no arguments to no peers") {
		bool done = false;
		a.bind("hello", [&done]() {
			done = true;
		});
		
		b.broadcast("done");
		
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}
	
	SECTION("no arguments to one peer") {
		b.connect("tcp://localhost:7077");
		while (a.numberOfPeers() == 0) std::this_thread::sleep_for(std::chrono::milliseconds(20));
		
		bool done = false;
		a.bind("hello", [&done]() {
			done = true;
		});
		
		b.broadcast("hello");
		
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
		
		REQUIRE( done );
	}
}

/*TEST_CASE("net::listen()", "[net]") {

	SECTION("tcp any interface") {
		REQUIRE( ftl::net::listen("tcp://localhost:9001")->isListening() );

		SECTION("can connect to listening socket") {
			auto sock = ftl::net::connect("tcp://127.0.0.1:9001");
			REQUIRE(sock->isValid());
			ftl::net::wait([&sock]() { return sock->isConnected(); });
			REQUIRE(sock->isConnected());

			// TODO Need way of knowing about connection
		}

		ftl::net::stop();
	}
	
	SECTION("on connection event") {
		auto l = ftl::net::listen("tcp://localhost:9002");
		REQUIRE( l->isListening() );
		
		bool connected = false;
		
		l->onConnection([&](shared_ptr<Socket> s) {
			ftl::net::wait([&s]() { return s->isConnected(); });
			REQUIRE( s->isConnected() );
			connected = true;
		});
		
		auto sock = ftl::net::connect("tcp://127.0.0.1:9002");
		ftl::net::wait();
		REQUIRE( connected );
		ftl::net::stop();
	}
}

TEST_CASE("Net Integration", "[integrate]") {
	std::string data;
	
	Protocol p("ftl://utu.fi");
	
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
	s2->setProtocol(&p);
	
	REQUIRE( s2 != nullptr );
	ftl::net::wait([&s2]() { return s2->isConnected(); });
	REQUIRE( s1 != nullptr );	

	REQUIRE( s1->isConnected() );
	REQUIRE( s2->isConnected() );
	
	REQUIRE( s1->call<int>("add", 5, 6) == 11 );
	REQUIRE( s2->call<int>("add", 10, 5) == 15);
	
	s1->send(100, "hello world");
	ftl::net::wait();
	// TODO s2->wait(100);
	
	REQUIRE( data == "hello world" );
}*/

