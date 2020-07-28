#include "catch.hpp"
#include <ftl/uri.hpp>
#include <nlohmann/json.hpp>

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

	GIVEN( "a valid fragment" ) {
		URI uri("http://localhost:8080/test/case.html#frag");

		REQUIRE( uri.isValid() );
		REQUIRE( uri.getScheme() == URI::SCHEME_HTTP );
		REQUIRE( uri.getHost() == "localhost" );
		REQUIRE( (uri.getPort() == 8080) );
		REQUIRE( uri.getPath() == "/test/case.html" );
		REQUIRE( uri.getFragment() == "frag");
	}

	GIVEN( "a multipart valid fragment" ) {
		URI uri("http://localhost:8080/test/case.html#frag/second");

		REQUIRE( uri.isValid() );
		REQUIRE( uri.getScheme() == URI::SCHEME_HTTP );
		REQUIRE( uri.getHost() == "localhost" );
		REQUIRE( (uri.getPort() == 8080) );
		REQUIRE( uri.getPath() == "/test/case.html" );
		REQUIRE( uri.getFragment() == "frag/second");
		REQUIRE( uri.getBaseURI() == "http://localhost:8080/test/case.html");
	}

	GIVEN( "an empty fragment" ) {
		URI uri("http://localhost:8080/test/case.html#");

		REQUIRE( uri.isValid() );
		REQUIRE( uri.getScheme() == URI::SCHEME_HTTP );
		REQUIRE( uri.getHost() == "localhost" );
		REQUIRE( (uri.getPort() == 8080) );
		REQUIRE( uri.getPath() == "/test/case.html" );
		REQUIRE( uri.getFragment() == "");
		REQUIRE( uri.getBaseURI() == "http://localhost:8080/test/case.html");
	}

	/*GIVEN( "a valid fragment with query" ) {
		URI uri("http://localhost:8080/test/case.html#frag?q=4");

		REQUIRE( uri.isValid() );
		REQUIRE( uri.getScheme() == URI::SCHEME_HTTP );
		REQUIRE( uri.getHost() == "localhost" );
		REQUIRE( (uri.getPort() == 8080) );
		REQUIRE( uri.getPath() == "/test/case.html" );
		REQUIRE( uri.getQuery() == "q=4" );
		REQUIRE( uri.getFragment() == "frag");
	}*/

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

SCENARIO( "URI::to_json() from a valid URI" ) {
	GIVEN( "no query component" ) {
		URI uri("http://localhost:1000/hello");

		nlohmann::json object;
		uri.to_json(object);

		REQUIRE( object["uri"].get<std::string>() == "http://localhost:1000/hello" );
	}

	GIVEN( "one numeric query item" ) {
		URI uri("http://localhost:1000/hello?a=45");

		nlohmann::json object;
		uri.to_json(object);

		REQUIRE( object["a"].get<int>() == 45 );
	}

	GIVEN( "multiple query items" ) {
		URI uri("http://localhost:1000/hello?a=45&b=world");

		nlohmann::json object;
		uri.to_json(object);

		REQUIRE( object["a"].get<int>() == 45 );
		REQUIRE( object["b"].get<std::string>() == "world" );
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

SCENARIO( "URI::getBaseURI(N)" ) {
	GIVEN( "an N of 0" ) {
		URI uri("http://localhost:1000/hello/world");
		REQUIRE( uri.getBaseURI(0) == "http://localhost:1000" );
	}

	GIVEN( "an N of -1" ) {
		URI uri("http://localhost:1000/hello/world");
		REQUIRE( uri.getBaseURI(-1) == "http://localhost:1000/hello" );
	}

	GIVEN( "an N of 1" ) {
		URI uri("http://localhost:1000/hello/world");
		REQUIRE( uri.getBaseURI(1) == "http://localhost:1000/hello" );
	}

	GIVEN( "an N of 2" ) {
		URI uri("http://localhost:1000/hello/world");
		REQUIRE( uri.getBaseURI(2) == "http://localhost:1000/hello/world" );
	}
}

SCENARIO( "URI::getBaseURIWithUser()" ) {
	GIVEN( "both username and password" ) {
		URI uri("http://nick:test@localhost:1000/hello/world?group=test2");
		REQUIRE( uri.getBaseURIWithUser() == "http://nick:test@localhost:1000/hello/world" );
	}

	GIVEN( "missing username and password" ) {
		URI uri("http://localhost:1000/hello/world?group=test2");
		REQUIRE( uri.getBaseURIWithUser() == "http://localhost:1000/hello/world" );
	}
}

