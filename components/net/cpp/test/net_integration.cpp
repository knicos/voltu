#include "catch.hpp"
#include <ftl/net.hpp>
#include <ftl/timer.hpp>

#include <thread>
#include <chrono>

using ftl::net::Universe;
using ftl::net::Peer;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;

// --- Support -----------------------------------------------------------------

// --- Tests -------------------------------------------------------------------

TEST_CASE("Universe::connect()", "[net]") {
	Universe a;
	Universe b;
	
	a.listen("tcp://localhost:7077");

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

TEST_CASE("Universe::onConnect()", "[net]") {
	Universe a;
	Universe b;
	
	a.listen("tcp://localhost:7077");

	SECTION("single valid remote init connection") {
		bool done = false;

		a.onConnect([&done](Peer *p) {
			done = true;
		});

		b.connect("tcp://localhost:7077")->waitConnection();
		sleep_for(milliseconds(100));
		REQUIRE( done );
	}

	SECTION("single valid init connection") {
		bool done = false;

		b.onConnect([&done](Peer *p) {
			done = true;
		});

		b.connect("tcp://localhost:7077")->waitConnection();
		//sleep_for(milliseconds(100));
		REQUIRE( done );
	}
}

TEST_CASE("Universe::onDisconnect()", "[net]") {
	Universe a;
	Universe b;

	a.listen("tcp://localhost:7077");

	SECTION("single valid remote close") {
		bool done = false;

		a.onDisconnect([&done](Peer *p) {
			done = true;
		});

		Peer *p = b.connect("tcp://localhost:7077");
		p->waitConnection();
		sleep_for(milliseconds(100));
		p->close();
		sleep_for(milliseconds(100));
		REQUIRE( done );
	}

	SECTION("single valid close") {
		bool done = false;

		b.onDisconnect([&done](Peer *p) {
			done = true;
		});

		Peer *p = b.connect("tcp://localhost:7077");
		p->waitConnection();
		sleep_for(milliseconds(100));
		p->close();
		sleep_for(milliseconds(100));
		REQUIRE( done );
	}
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
		
		sleep_for(milliseconds(50));
		REQUIRE( !done );
	}
	
	SECTION("no arguments to one peer") {
		b.connect("tcp://localhost:7077")->waitConnection();
		
		bool done = false;
		a.bind("hello", [&done]() {
			done = true;
		});
		
		b.broadcast("hello");
		
		while (!done) sleep_for(milliseconds(5));
		
		REQUIRE( done );
	}
	
	SECTION("one argument to one peer") {
		b.connect("tcp://localhost:7077")->waitConnection();
		
		int done = 0;
		a.bind("hello", [&done](int v) {
			done = v;
		});
		
		b.broadcast("hello", 676);
		
		while (done == 0) sleep_for(milliseconds(5));
		
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

		REQUIRE( a.numberOfPeers() == 2 );
		//sleep_for(milliseconds(100)); // NOTE: Binding might not be ready
		
		a.broadcast("hello", 676);
		
		sleep_for(milliseconds(100));
		
		REQUIRE( done1 == 676 );
		REQUIRE( done2 == 676 );
	}
}

TEST_CASE("Universe::findAll()", "") {
	Universe a;
	Universe b;
	Universe c;

	a.listen("tcp://localhost:7077");
	b.connect("tcp://localhost:7077")->waitConnection();
	c.connect("tcp://localhost:7077")->waitConnection();

	SECTION("no values exist") {
		REQUIRE( (c.findAll<int>("test_all").size() == 0) );
	}

	SECTION("one set exists") {
		a.bind("test_all", []() -> std::vector<int> {
			return {3,4,5};
		});

		auto res = c.findAll<int>("test_all");
		REQUIRE( (res.size() == 3) );
		REQUIRE( (res[0] == 3) );
	}

	SECTION("two sets exists") {
		b.bind("test_all", []() -> std::vector<int> {
			return {3,4,5};
		});
		c.bind("test_all", []() -> std::vector<int> {
			return {6,7,8};
		});

		//sleep_for(milliseconds(100)); // NOTE: Binding might not be ready

		auto res = a.findAll<int>("test_all");
		REQUIRE( (res.size() == 6) );
		REQUIRE( (res[0] == 3 || res[0] == 6) );
	}
}

TEST_CASE("Peer::call() __ping__", "") {
	Universe a;
	Universe b;
	Universe c;

	a.listen("tcp://localhost:7077");
	auto *p = b.connect("tcp://localhost:7077");
	p->waitConnection();

	SECTION("single ping") {
		int64_t res = p->call<int64_t>("__ping__");
		REQUIRE((res <= ftl::timer::get_time() && res > 0));
	}

	SECTION("large number of pings") {
		for (int i=0; i<100; ++i) {
			int64_t res = p->call<int64_t>("__ping__");
			REQUIRE(res > 0);
		}
	}

	SECTION("large number of parallel pings") {
		std::atomic<int> count = 0;
		for (int i=0; i<100; ++i) {
			ftl::pool.push([&count, p](int id) {
				int64_t res = p->call<int64_t>("__ping__");
				REQUIRE( res > 0 );
				count++;
			});
		}

		while (count < 100) std::this_thread::sleep_for(milliseconds(5));
	}

	SECTION("single invalid rpc") {
		bool errored = false;
		try {
			int64_t res = p->call<int64_t>("__ping2__");
			REQUIRE( res > 0 );  // Not called or required actually
		} catch (const ftl::exception &e) {
			errored = true;
		}
		REQUIRE(errored);
	}
}
