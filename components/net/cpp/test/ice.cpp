#include "catch.hpp"
#include <string.h>
#include <ftl/net/ice.hpp>
#include <iostream>

TEST_CASE( "net::ice::stun()", "[ice][net]" ) {
	std::string c;

	SECTION( "manual stun server" ) {
		REQUIRE( ftl::net::ice::stun(c, "udp://stun.l.google.com:19302", 7079) == 0 );
	}

	SECTION( "automatic stun server" ) {
		REQUIRE( ftl::net::ice::stun(c, 7079, false) == 0 );
	}

	std::cerr << "STUN Result: " << c << std::endl;
	REQUIRE(c.size() > 0);
}

TEST_CASE( "net::ice::candidates()", "[ice][net]" ) {
	std::vector<std::string> cc;

	SECTION( "udp candidate list" ) {
		ftl::net::ice::candidates(cc, 7079, false);
	}

	REQUIRE( cc.size() >= 2 );
	for (auto x : cc) std::cerr << "Candidate: " << x << std::endl;
}
