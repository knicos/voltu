#include "catch.hpp"
#include "costs/census.hpp"
#include "costs/gradient.hpp"
#include "util_opencv.hpp"

TEST_CASE("Gradient Matching Cost", "") {
	SECTION("Construct a Gradient matching cost") {
		GradientMatchingCostL2 dsi(100,100,10,20);

		cv::Mat left(100,100, CV_8UC1);
		cv::Mat right(100,100, CV_8UC1);

		left.setTo(cv::Scalar(0));
		right.setTo(cv::Scalar(0));

		Array2D<uchar> l(left);
		Array2D<uchar> r(right);
		dsi.set(l, r);

		REQUIRE( dsi.data()(5,5,2) == 0 );
	}
}

TEST_CASE("Census Matching Cost", "") {
	SECTION("Construct a Census matching cost") {
		CensusMatchingCost dsi(100,100,10,20,9,7);

		cv::Mat left(100,100, CV_8UC1);
		cv::Mat right(100,100, CV_8UC1);

		left.setTo(cv::Scalar(0));
		right.setTo(cv::Scalar(0));

		Array2D<uchar> l(left);
		Array2D<uchar> r(right);
		dsi.set(l, r);

		REQUIRE( dsi.data()(5,5,2) == 0 );
	}
}
