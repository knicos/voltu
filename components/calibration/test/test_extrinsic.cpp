#include "catch.hpp"

#include <ftl/calibration/visibility.hpp>

using std::vector;
using ftl::calibration::dijstra;

/// https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-greedy-algo-7/
static const vector<vector<int>> graph1 = {
		{0, 4, 0, 0, 0, 0, 0, 8, 0},
		{4, 0, 8, 0, 0, 0, 0,11, 0},
		{0, 8, 0, 7, 0, 4, 0, 0, 2},
		{0, 0, 7, 0, 9,14, 0, 0, 0},
		{0, 0, 0, 9, 0,10, 0, 0, 0},
		{0, 0, 4,14,10, 0, 2, 0, 0},
		{0, 0, 0, 0, 0, 2, 0, 1, 6},
		{8,11, 0, 0, 0, 0, 1, 0, 7},
		{0, 0, 2, 0, 0, 0, 6, 7, 0}
};

TEST_CASE("Dijstra's Algorithm", "") {
	SECTION("Find shortest paths") {
		auto path = dijstra(0, graph1);

		REQUIRE(path.distance(1) == 4);
		REQUIRE(path.distance(2) == 12);
		REQUIRE(path.distance(3) == 19);
		REQUIRE(path.distance(4) == 21);
		REQUIRE(path.distance(5) == 11);
		REQUIRE(path.distance(6) == 9);
		REQUIRE(path.distance(7) == 8);
		REQUIRE(path.distance(8) == 14);

		REQUIRE((path.to(1) == vector {0, 1}));
		REQUIRE((path.to(2) == vector {0, 1, 2}));
		REQUIRE((path.to(3) == vector {0, 1, 2, 3}));
		REQUIRE((path.to(4) == vector {0, 7, 6, 5, 4}));
		REQUIRE((path.to(5) == vector {0, 7, 6, 5}));
		REQUIRE((path.to(6) == vector {0, 7, 6}));
		REQUIRE((path.to(7) == vector {0, 7}));
		REQUIRE((path.to(8) == vector {0, 1, 2, 8}));
	}

	SECTION("Check connectivity") {
		auto path = dijstra(0, graph1);
		REQUIRE(path.connected());
	}
}
