#include "catch.hpp"
#include <ftl/net.hpp>

#include <thread>
#include <chrono>

using ftl::net::Universe;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;

// --- Support -----------------------------------------------------------------

// --- Tests -------------------------------------------------------------------

TEST_CASE("Universe::connect()", "[net]") {
	Universe a;
	Universe b;
	
	a.listen("tcp://localhost:7077");
	//sleep_for(milliseconds(100));

	SECTION("valid tcp connection using ipv4") {
		auto p = b.connect("tcp://127.0.0.1:7077");
		REQUIRE( p );
		
		p->waitConnection();
		
		REQUIRE( a.numberOfPeers() == 1 );
		REQUIRE( b.numberOfPeers() == 1 );
	}

	SECTION("valid tcp connection using hostname") {
		auto p = b.connect("tcp://localhost:7077");
		REQUIRE( p );
		
		p->waitConnection();
		
		REQUIRE( a.numberOfPeers() == 1 );
		REQUIRE( b.numberOfPeers() == 1 );
	}

	SECTION("invalid protocol") {
		auto p = b.connect("http://127.0.0.1:7077");
		REQUIRE( !p->isValid() );
		
		sleep_for(milliseconds(100));
		
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
	Universe a;
	Universe b;
	
	a.listen("tcp://localhost:7077");
	
	SECTION("no arguments to no peers") {
		bool done = false;
		a.bind("hello", [&done]() {
			done = true;
		});
		
		b.broadcast("done");
		
		sleep_for(milliseconds(100));
		REQUIRE( !done );
	}
	
	SECTION("no arguments to one peer") {
		b.connect("tcp://localhost:7077")->waitConnection();
		
		bool done = false;
		a.bind("hello", [&done]() {
			done = true;
		});
		
		b.broadcast("hello");
		
		sleep_for(milliseconds(100));
		
		REQUIRE( done );
	}
	
	SECTION("one argument to one peer") {
		b.connect("tcp://localhost:7077")->waitConnection();
		
		int done = 0;
		a.bind("hello", [&done](int v) {
			done = v;
		});
		
		b.broadcast("hello", 676);
		
		sleep_for(milliseconds(100));
		
		REQUIRE( done == 676 );
	}
	
	SECTION("one argument to two peers") {
		Universe c;
		
		b.connect("tcp://localhost:7077")->waitConnection();
		c.connect("tcp://localhost:7077")->waitConnection();
		
		int done1 = 0;
		b.bind("hello", [&done1](int v) {
			done1 = v;
		});
		
		int done2 = 0;
		c.bind("hello", [&done2](int v) {
			done2 = v;
		});
		
		a.broadcast("hello", 676);
		
		sleep_for(milliseconds(100));
		
		REQUIRE( done1 == 676 );
		REQUIRE( done2 == 676 );
	}
}

TEST_CASE("Universe::findOwner()", "") {
	Universe a;
	Universe b;
	a.listen("tcp://localhost:7077");
	b.connect("tcp://localhost:7077")->waitConnection();

	SECTION("no owners exist") {
		REQUIRE( !b.findOwner("ftl://test") );
	}

	SECTION("one owner exists") {
		a.createResource("ftl://test");
		REQUIRE( *(b.findOwner("ftl://test")) == ftl::net::this_peer );
	}
	
	SECTION("three peers and one owner") {
		Universe c;
		c.connect("tcp://localhost:7077")->waitConnection();
		b.setLocalID(ftl::UUID(7));

		b.createResource("ftl://test");
		REQUIRE( *(a.findOwner("ftl://test")) == ftl::UUID(7) );
	}

	SECTION("three peers and one owner (2)") {
		Universe c;
		c.connect("tcp://localhost:7077")->waitConnection();
		c.setLocalID(ftl::UUID(7));

		c.createResource("ftl://test");
		auto r = a.findOwner("ftl://test");
		REQUIRE( r );
		REQUIRE( *r == ftl::UUID(7) );
	}
}

TEST_CASE("Universe::subscribe()", "") {
	Universe a;
	Universe b;
	a.listen("tcp://localhost:7077");
	b.connect("tcp://localhost:7077")->waitConnection();

	SECTION("no resource exists") {
		REQUIRE( !b.subscribe("ftl://test", []() {}) );
	}

	SECTION("one resource exists") {
		a.createResource("ftl://test");
		REQUIRE( b.subscribe("ftl://test", []() {}) );
		sleep_for(milliseconds(50));
		REQUIRE( a.numberOfSubscribers("ftl://test") == 1);
	}
}

TEST_CASE("Universe::publish()", "") {
	Universe a;
	Universe b;
	a.listen("tcp://localhost:7077");
	b.connect("tcp://localhost:7077")->waitConnection();

	SECTION("no subscribers") {
		a.createResource("ftl://test");
		a.publish("ftl://test", 55);
	}

	SECTION("one subscriber") {
		int done = 0;
		a.createResource("ftl://test");
		REQUIRE( b.subscribe("ftl://test", [&done](int a) {
			done = a;
		}) );
		sleep_for(milliseconds(50));

		a.publish("ftl://test", 56);
		sleep_for(milliseconds(50));
		
		REQUIRE( done == 56 );
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
