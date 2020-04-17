#include "catch.hpp"
#include "../src/dsi.hpp"
#include "../src/wta.hpp"
#include "../src/util.hpp"
#include "../src/consistency.hpp"

using algorithms::WTA;
using algorithms::WTADiagonal;
using algorithms::ConsistencyCheck;

TEST_CASE("Winner Takes All", "") {
	const int w = 100;
	const int h = 100;
	const int dmin = 10;
	const int dmax = 20;
	const int d_true = 12;

	DSImage16U dsi(w, h, dmin, dmax);
	Array2D<float> disparity(w, h);
	Array2D<unsigned short> min_cost(w, h);

	// d(y,x,d) == 0 iff d == d_true, otherwise 100
	for (int y = 0; y < h; y++) {
		for (int x = 0; x < w; x++) {
			for (int d = dmin; d <= dmax; d++) {
				dsi.data()(y,x,d) = (d == d_true) ? 0 : 100;
			}
		}
	}

	SECTION("WTA from a ushort DSI") {
		WTA<DSImage16U, float, 0> wta = {dsi.data(), disparity.data(), min_cost.data()};
		parallel2D(wta, dsi.width(), dsi.height());
		int valid = 0;
		for (int y = 0; y < h; y++) {
			for (int x = 0; x < w; x++) {
				valid += (float(d_true) == disparity.data()(y,x)) ? 1 : 0;
			}
		}
		REQUIRE(valid == w*h);
	}

	SECTION("WTA diagonal from a ushort DSI") {
		WTADiagonal<DSImage16U,float> wta = {dsi.data(), disparity.data()};
		parallel2D(wta, dsi.width(), dsi.height());

		int valid = 0;
		for (int y = 0; y < h; y++) {
			for (int x = 0; x < w; x++) {
				valid += (float(d_true) == disparity.data()(y,x)) ? 1 : 0;
			}
		}
	}

	SECTION("WTA with consistency check") {
		WTA<DSImage16U, float, 0> wta = {dsi.data(), disparity.data(), min_cost.data()};
		parallel2D(wta, dsi.width(), dsi.height());

		Array2D<float> disparityr(100,100);

		WTADiagonal<DSImage16U, float> wtadiag = {dsi.data(), disparityr.data()};
		parallel2D(wtadiag, dsi.width(), dsi.height());

		parallel2D<ConsistencyCheck<float>>({disparity.data(), disparityr.data()}, dsi.width(), dsi.height());
	}
}

TEST_CASE("WTA Wrapper") {
	WinnerTakesAll<DSImage16U, float> wta;
	DSImage16U dsi(100,100,10,20);
	wta(dsi);
}
