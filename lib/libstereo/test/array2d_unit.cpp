#include "catch.hpp"
#include "../src/array2d.hpp"

TEST_CASE("Array2D", "") {
	SECTION("Construct a float array2d") {
		Array2D<float> array(100,100);

		array.data()(5,5) = 677;
		REQUIRE( array.data()(5,5) == 677 );
	}
}
