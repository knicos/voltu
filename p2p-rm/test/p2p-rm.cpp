#include "catch.hpp"
#include <ftl/p2p-rm.hpp>
#include <ftl/net/socket.hpp>
#include <vector>
#include <iostream>

// --- Mock Socket Send

static std::vector<uint32_t> msgs;

int ftl::net::Socket::send2(uint32_t service, const std::string &data1, const std::string &data2) {
	msgs.push_back(service);
	std::cout << "SEND2 (" << service << ")" << std::endl;
	return 0;
}

// --- End Mock Socket Send

SCENARIO( "Cluster::map()", "[map]" ) {
	auto cluster = ftl::rm::cluster("ftl://utu.fi", nullptr);
	
	GIVEN( "a valid URI and array datatype" ) {
		int data[10];
		auto m = cluster->map<int[10]>("ftl://utu.fi/memory/test0", &data);
		REQUIRE( m.is_valid() );

		auto r = cluster->get<int[10]>("ftl://utu.fi/memory/test0");
		REQUIRE( r.is_valid() );
		REQUIRE( r.size() == 10*sizeof(int) );
		REQUIRE( r.is_local() );
	}
	
	GIVEN( "a valid URI and invalid data" ) {
		auto m = cluster->map<int>("ftl://utu.fi/memory/test0", NULL);
		REQUIRE( !m.is_valid() );
	}
	
	GIVEN( "an empty URI" ) {
		int data;
		auto m = cluster->map<int>("", &data);
		REQUIRE( !m.is_valid() );
	}
	
	GIVEN( "an invalid URI" ) {
		int data;
		auto m = cluster->map<int>("noschema/test", &data);
		REQUIRE( !m.is_valid() );
	}
	
	GIVEN( "an invalid URI schema" ) {
		int data;
		auto m = cluster->map<int>("http://utu.fi/memory/test0", &data);
		REQUIRE( !m.is_valid() );
	}
	
	GIVEN( "an invalid URI host" ) {
		int data;
		auto m = cluster->map<int>("ftl://yle.fi/wrong/test0", &data);
		REQUIRE( !m.is_valid() );
	}
	
	GIVEN( "a duplicate URI" ) {
		int data;
		auto a = cluster->map<int>("ftl://utu.fi/memory/test0", &data);
		auto b = cluster->map<int>("ftl://utu.fi/memory/test0", &data);
		REQUIRE( !b.is_valid() );
		REQUIRE( a.is_valid() );
	}
}

SCENARIO( "Getting a read_ref", "[get]" ) {
	auto cluster = ftl::rm::cluster("ftl://utu.fi", nullptr);
	int data = 89;
	auto m = cluster->map<int>("ftl://utu.fi/memory/test1", &data);
	REQUIRE( m.is_valid() );
	
	GIVEN( "a valid URI to local memory" ) {
		const auto r = cluster->getReadable<int>("ftl://utu.fi/memory/test1");
		REQUIRE( r.is_valid() );
		REQUIRE( r.pointer().is_local() );
		REQUIRE( r == 89 );
	}
	
	GIVEN( "a valid URI to remote memory" ) {
		const auto r = cluster->getReadable<int>("ftl://utu.fi/memory/remote0");
		REQUIRE( r.is_valid() );
		REQUIRE( !r.pointer().is_local() );
		REQUIRE( r == 888 );
	}
}

