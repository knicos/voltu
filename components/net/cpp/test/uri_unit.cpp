#include "catch.hpp"
#include <ftl/uri.hpp>

using ftl::URI;
using std::string;

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
		REQUIRE( (uri.getPort() == 8080) );
		REQUIRE( uri.getPath() == "/test/case.html" );
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

SCENARIO( "URI::to_string() from a valid URI" ) {
	GIVEN( "no query component" ) {
		URI uri("http://localhost:1000/hello");
		REQUIRE( uri.to_string() == "http://localhost:1000/hello" );
	}

	GIVEN( "A single query component" ) {
		URI uri("http://localhost:1000/hello?x=5");
		REQUIRE( uri.to_string() == "http://localhost:1000/hello?x=5" );
	}

	GIVEN( "an unsorted set of query components" ) {
		URI uri("http://localhost:1000/hello?z=5&y=4&x=2");
		REQUIRE( uri.to_string() == "http://localhost:1000/hello?x=2&y=4&z=5" );
	}
}

SCENARIO( "URI::getAttribute() from query" ) {
	GIVEN( "a string value" ) {
		URI uri("http://localhost:1000/hello?x=world");
		REQUIRE( uri.getAttribute<string>("x") == "world" );
	}

	GIVEN( "an integer value" ) {
		URI uri("http://localhost:1000/hello?x=56");
		REQUIRE( (uri.getAttribute<int>("x") == 56) );
	}
}

