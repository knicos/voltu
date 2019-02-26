#include "catch.hpp"
#include <ftl/p2p-rm/cluster.hpp>
#include <ftl/net/socket.hpp>
#include <ftl/net.hpp>
#include <vector>
#include <iostream>

using ftl::rm::Cluster;

// --- Mock Socket Send

/*static std::vector<uint32_t> msgs;

int ftl::net::Socket::send2(uint32_t service, const std::string &data1, const std::string &data2) {
	msgs.push_back(service);
	std::cout << "SEND2 (" << service << ")" << std::endl;
	return 0;
}

ftl::net::Socket::Socket(int s) : disp_(this) {

}

ftl::net::Socket::~Socket() {

}

int ftl::net::Socket::rpcid__ = 0;

int ftl::net::Socket::send(uint32_t service, const std::string &data) {
	msgs.push_back(service);
	std::cout << "SEND (" << service << ")" << std::endl;

	return 0;
}

bool ftl::net::wait() {
	return true;
}

std::shared_ptr<ftl::net::Socket> ftl::net::connect(const char *url) {
	return nullptr;
}*/

// --- End Mock Socket Send

SCENARIO( "Cluster::map()", "[map]" ) {
	Cluster cluster("ftl://utu.fi", nullptr);
	
	GIVEN( "a valid URI and array datatype" ) {
		int data[10];
		auto m = cluster.map<int[10]>("ftl://utu.fi/memory/test0", &data);
		REQUIRE( m.is_valid() );

		auto r = cluster.get<int[10]>("ftl://utu.fi/memory/test0");
		REQUIRE( r.is_valid() );
		REQUIRE( r.size() == 10*sizeof(int) );
		REQUIRE( r.is_local() );
	}
	
	GIVEN( "a valid URI and invalid data" ) {
		auto m = cluster.map<int>("ftl://utu.fi/memory/test0", NULL);
		REQUIRE( !m.is_valid() );
	}
	
	GIVEN( "an empty URI" ) {
		int data;
		auto m = cluster.map<int>("", &data);
		REQUIRE( !m.is_valid() );
	}
	
	GIVEN( "an invalid URI" ) {
		int data;
		auto m = cluster.map<int>("noschema/test", &data);
		REQUIRE( !m.is_valid() );
	}
	
	GIVEN( "an invalid URI schema" ) {
		int data;
		auto m = cluster.map<int>("http://utu.fi/memory/test0", &data);
		REQUIRE( !m.is_valid() );
	}
	
	GIVEN( "an invalid URI host" ) {
		int data;
		auto m = cluster.map<int>("ftl://yle.fi/wrong/test0", &data);
		REQUIRE( !m.is_valid() );
	}
	
	GIVEN( "a duplicate URI" ) {
		int data;
		auto a = cluster.map<int>("ftl://utu.fi/memory/test0", &data);
		auto b = cluster.map<int>("ftl://utu.fi/memory/test0", &data);
		REQUIRE( !b.is_valid() );
		REQUIRE( a.is_valid() );
	}
}

SCENARIO( "Getting a read_ref", "[get]" ) {
	Cluster cluster("ftl://utu.fi", nullptr);
	// Add fake peer
	auto p = std::make_shared<ftl::net::Socket>(0);
	cluster.addPeer(p);
	
	int data = 89;
	int data2 = 99;
	auto m = cluster.map<int>("ftl://utu.fi/memory/test1", &data);
	cluster.map<int>("ftl://utu.fi/memory/remote0", &data2);
	REQUIRE( m.is_valid() );
	
	GIVEN( "a valid URI to local memory" ) {
		const auto r = cluster.getReadable<int>("ftl://utu.fi/memory/test1");
		REQUIRE( r.is_valid() );
		REQUIRE( r.pointer().is_local() );
		REQUIRE( r == 89 );
	}
	
	GIVEN( "a valid URI to remote memory" ) {
		const auto r = cluster.getReadable<int>("ftl://utu.fi/memory/remote0");
		REQUIRE( r.is_valid() );
		//REQUIRE( !r.pointer().is_local() );
		REQUIRE( r == 888 );
	}
}

