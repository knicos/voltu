#include "catch.hpp"
#include <ftl/p2p-rm/cluster.hpp>
#include <ftl/net/socket.hpp>
#include <ftl/net/listener.hpp>
#include <ftl/net.hpp>
#include <vector>
#include <iostream>

using ftl::rm::Cluster;

SCENARIO( "Pre-connection ownership resolution", "[ownership]" ) {
	Cluster c1("ftl://utu.fi", nullptr);
	auto l = ftl::net::listen("tcp://localhost:9000");
	l->setProtocol(&c1);
	Cluster c2("ftl://utu.fi", l);
	
	auto s = c1.addPeer("tcp://localhost:9000");
	
	int data1 = 89;
	int data2 = 99;
	
	auto m1 = c1.map<int>("ftl://utu.fi/memory/r1", &data1);
	auto m2 = c2.map<int>("ftl://utu.fi/memory/r1", &data2);
	REQUIRE( m1.is_valid() );
	REQUIRE( m2.is_valid() );
	
	ftl::net::wait([&s]() { return s->isConnected(); });
	
	REQUIRE( m2.is_owner() );
	REQUIRE( !m1.is_owner() );
	
	l->close();
	ftl::net::stop();
}

	
SCENARIO( "Post-connection ownership resolution", "[ownership]" ) {
	Cluster c1("ftl://utu.fi", nullptr);
	auto l = ftl::net::listen("tcp://localhost:9000");
	l->setProtocol(&c1);
	Cluster c2("ftl://utu.fi", l);
	
	auto s = c1.addPeer("tcp://localhost:9000");
	
	int data1 = 89;
	int data2 = 99;
	
	ftl::net::wait([&s]() { return s->isConnected(); });
	
	auto m1 = c1.map<int>("ftl://utu.fi/memory/r1", &data1);
	auto m2 = c2.map<int>("ftl://utu.fi/memory/r1", &data2);
	REQUIRE( m1.is_valid() );
	REQUIRE( m2.is_valid() );
	
	REQUIRE( !m2.is_owner() );
	REQUIRE( m1.is_owner() );
	
	l->close();
	ftl::net::stop();
}
	
SCENARIO( "Write change ownership", "[ownership]" ) {
	Cluster c1("ftl://utu.fi", nullptr);
	auto l = ftl::net::listen("tcp://localhost:9000");
	l->setProtocol(&c1);
	Cluster c2("ftl://utu.fi", l);
	
	auto s = c1.addPeer("tcp://localhost:9000");
	
	int data1 = 89;
	int data2 = 99;
	
	ftl::net::wait([&s]() { return s->isConnected(); });
	
	auto m1 = c1.map<int>("ftl://utu.fi/memory/r1", &data1);
	auto m2 = c2.map<int>("ftl://utu.fi/memory/r1", &data2);
	REQUIRE( m1.is_valid() );
	REQUIRE( m2.is_valid() );
	
	REQUIRE( !m2.is_owner() );
	REQUIRE( m1.is_owner() );
	
	*m2 = 676;
	
	REQUIRE( m2.is_owner() );
	REQUIRE( !m1.is_owner() );
	
	l->close();
	ftl::net::stop();
}

	
	/*
	
	REQUIRE( *m2 == 99 );
	REQUIRE( *m1 == 89 );
	
	*m2 = 77;
	ftl::net::wait();
	REQUIRE( *m2 == 77 );
	REQUIRE( *m1 == 77 );
	
	*m1 = 66;
	REQUIRE( *m2 == 66 );
	REQUIRE( *m1 == 66 );*/
