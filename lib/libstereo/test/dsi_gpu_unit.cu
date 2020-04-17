#include "catch.hpp"
#include "../src/dsi.hpp"

TEST_CASE("DisparitySpaceImage", "") {
	SECTION("Construct a ushort DSI") {
		DisparitySpaceImage<unsigned short> dsi(100,100,10,20);

		dsi.set(677);
		REQUIRE( dsi(5,5,2) == 677 );
	}
}
