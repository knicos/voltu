#include "catch.hpp"
#include <ftl/p2p-rm.hpp>

SCENARIO( "ftl::rm::alloc()", "[alloc]" ) {
	ftl::rm::reset();
	
	GIVEN( "a valid URI and size" ) {
		auto r = ftl::rm::alloc<int>("ftl://uti.fi/memory/test0", 10);
		REQUIRE( r.is_valid() );
		REQUIRE( r.size() == 10*sizeof(int) );
		REQUIRE( r.is_local() );
	}
	
	GIVEN( "a valid URI and invalid size" ) {
		auto r = ftl::rm::alloc<int>("ftl://uti.fi/memory/test0", 0);
		REQUIRE( !r.is_valid() );
	}
	
	GIVEN( "an empty URI" ) {
		auto r = ftl::rm::alloc<int>("", 10);
		REQUIRE( !r.is_valid() );
	}
	
	GIVEN( "an invalid URI" ) {
		auto r = ftl::rm::alloc<int>("noschema/test", 10);
		REQUIRE( !r.is_valid() );
	}
	
	GIVEN( "an invalid URI schema" ) {
		auto r = ftl::rm::alloc<int>("http://uti.fi/memory/test0", 10);
		REQUIRE( !r.is_valid() );
	}
	
	GIVEN( "an invalid URI path segment" ) {
		auto r = ftl::rm::alloc<int>("ftl://uti.fi/wrong/test0", 10);
		REQUIRE( !r.is_valid() );
	}
	
	GIVEN( "a duplicate URI" ) {
		auto a = ftl::rm::alloc<int>("ftl://uti.fi/memory/test0", 10);
		auto b = ftl::rm::alloc<int>("ftl://uti.fi/memory/test0", 10);
		REQUIRE( a.is_valid() );
		REQUIRE( !b.is_valid() );
	}
}

SCENARIO( "Getting a read_ref", "[get]" ) {
	ftl::rm::reset();
	auto a = ftl::rm::alloc<int>("ftl://uti.fi/memory/test1", 10);
	REQUIRE( a.is_valid() );
	
	GIVEN( "a valid URI to local memory" ) {
		const auto r = ftl::rm::getReadable<int>("ftl://uti.fi/memory/test1");
		REQUIRE( r.is_valid() );
		REQUIRE( r.pointer().is_local() );
		REQUIRE( r.pointer().blob == a.blob );
	}
}

