#include "catch.hpp"
#include <ftl/p2p-rm.hpp>

// --- Mock Socket Send

int ftl::net::raw::Socket::send2(uint32_t service, const std::string &data1, const std::string &data2) {
	return 0;
}

// --- End Mock Socket Send

SCENARIO( "ftl::rm::map()", "[map]" ) {
	ftl::rm::reset();
	
	GIVEN( "a valid URI and array datatype" ) {
		int data[10];
		auto m = ftl::rm::map<int[10]>("ftl://uti.fi/memory/test0", &data);
		REQUIRE( m.is_valid() );

		auto r = ftl::rm::get<int[10]>("ftl://uti.fi/memory/test0");
		REQUIRE( r.is_valid() );
		REQUIRE( r.size() == 10*sizeof(int) );
		REQUIRE( r.is_local() );
	}
	
	GIVEN( "a valid URI and invalid data" ) {
		auto m = ftl::rm::map<int>("ftl://uti.fi/memory/test0", NULL);
		REQUIRE( !m.is_valid() );
	}
	
	GIVEN( "an empty URI" ) {
		int data;
		auto m = ftl::rm::map<int>("", &data);
		REQUIRE( !m.is_valid() );
	}
	
	GIVEN( "an invalid URI" ) {
		int data;
		auto m = ftl::rm::map<int>("noschema/test", &data);
		REQUIRE( !m.is_valid() );
	}
	
	GIVEN( "an invalid URI schema" ) {
		int data;
		auto m = ftl::rm::map<int>("http://uti.fi/memory/test0", &data);
		REQUIRE( !m.is_valid() );
	}
	
	GIVEN( "an invalid URI path segment" ) {
		int data;
		auto m = ftl::rm::map<int>("ftl://uti.fi/wrong/test0", &data);
		REQUIRE( !m.is_valid() );
	}
	
	GIVEN( "a duplicate URI" ) {
		int data;
		auto a = ftl::rm::map<int>("ftl://uti.fi/memory/test0", &data);
		auto b = ftl::rm::map<int>("ftl://uti.fi/memory/test0", &data);
		REQUIRE( !b.is_valid() );
		REQUIRE( a.is_valid() );
	}
}

SCENARIO( "Getting a read_ref", "[get]" ) {
	ftl::rm::reset();
	int data = 89;
	auto m = ftl::rm::map<int>("ftl://uti.fi/memory/test1", &data);
	REQUIRE( m.is_valid() );
	
	GIVEN( "a valid URI to local memory" ) {
		const auto r = ftl::rm::getReadable<int>("ftl://uti.fi/memory/test1");
		REQUIRE( r.is_valid() );
		REQUIRE( r.pointer().is_local() );
		REQUIRE( r == 89 );
	}
}

