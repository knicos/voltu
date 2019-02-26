#include "catch.hpp"
#include <ftl/uri.hpp>

using ftl::URI;

SCENARIO( "URI() can parse valid URIs", "[utility]" ) {
	GIVEN( "a valid scheme, no port or path" ) {
		URI uri("http://localhost");

		REQUIRE( uri.isValid() );
		REQUIRE( uri.getScheme() == URI::SCHEME_HTTP );
		REQUIRE( uri.getHost() == "localhost" );
		REQUIRE( uri.getPort() == 0 );
		REQUIRE (uri.getPath() == "" );
	}

	GIVEN( "a valid scheme with port and path" ) {
		URI uri("http://localhost:8080/test/case.html");

		REQUIRE( uri.isValid() );
		REQUIRE( uri.getScheme() == URI::SCHEME_HTTP );
		REQUIRE( uri.getHost() == "localhost" );
		REQUIRE( uri.getPort() == 8080 );
		REQUIRE (uri.getPath() == "/test/case.html" );
	}

	GIVEN( "a valid scheme with path and query" ) {
		URI uri("ftl://utu.fi/test/case.html?v=1");

		REQUIRE( uri.isValid() );
		REQUIRE( uri.getScheme() == URI::SCHEME_FTL );
		REQUIRE( uri.getHost() == "utu.fi" );
		REQUIRE( uri.getPath() == "/test/case.html" );
		REQUIRE( uri.getQuery() == "v=1" );
		REQUIRE( uri.getBaseURI() == "ftl://utu.fi/test/case.html" );
	}
}

SCENARIO( "URI() fails gracefully with invalid URIs", "[utility]" ) {
	GIVEN( "an invalid scheme" ) {
		URI uri("@://localhost:8080/test/case.html");
		REQUIRE( uri.getScheme() == URI::SCHEME_NONE );
		REQUIRE( !uri.isValid() );
	}

	GIVEN( "an invalid port" ) {
		URI uri("http://localhost:k/test/case.html");
		REQUIRE( !uri.isValid() );
	}

	GIVEN( "an empty URI" ) {
		URI uri("");
		REQUIRE( !uri.isValid() );
	}

	GIVEN( "only a scheme" ) {
		URI uri("tcp:");
		REQUIRE( !uri.isValid() );
	}
}

