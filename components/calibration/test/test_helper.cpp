#include "catch.hpp"
#include <ftl/calibration/extrinsic.hpp>

TEST_CASE("Exceptions") {
	SECTION("Require target is set before adding poitns") {
		auto helper = ftl::calibration::CalibrationPoints<double>();
		REQUIRE_THROWS(helper.addPoints(0, {{0,0}}));
	}

	SECTION("Do not allow setting points twice before next()") {
		auto helper = ftl::calibration::CalibrationPoints<double>();
		helper.setObject({{0, 0, 0}});
		helper.addPoints(0, {{0,0}});
		REQUIRE_THROWS(helper.addPoints(0, {{0,0}}));
	}

	SECTION("Adding points must have same number of points as in target") {
		auto helper = ftl::calibration::CalibrationPoints<double>();
		helper.setObject({{0, 0, 0}});
		REQUIRE_THROWS(helper.addPoints(0, {{0,0}, {0,0}}));
	}
}

TEST_CASE("Add and retrieve points") {
	SECTION("All same (double)") {
		auto helper = ftl::calibration::CalibrationPoints<double>();
		int npoints = 2;
		std::vector<cv::Point2d> points0 = {{0, 0}, {0, 1}};
		std::vector<cv::Point2d> points1 = {{0, 2}, {0, 3}};
		std::vector<cv::Point2d> points2 = {{0, 4}, {0, 5}};

		helper.setObject({{1,2,3}, {4,5,6}});
		helper.addPoints(0, points0);
		helper.addPoints(1, points1);
		helper.addPoints(2, points2);
		helper.next();

		auto points = helper.getPoints({0, 1, 2}, 0);

		REQUIRE(points.size() == 3);
		for (int i = 0; i < npoints; i++) {
			REQUIRE(points.at(0).at(i) == points0[i]);
			REQUIRE(points.at(1).at(i) == points1[i]);
			REQUIRE(points.at(2).at(i) == points2[i]);
		}
	}

	SECTION("One missing in first set, all queried (double)") {
		auto helper = ftl::calibration::CalibrationPoints<double>();
		int npoints = 2;
		std::vector<cv::Point2d> points0 = {{0, 0}, {0, 1}};
		std::vector<cv::Point2d> points1 = {{0, 2}, {0, 3}};
		std::vector<cv::Point2d> points2 = {{0, 4}, {0, 5}};

		helper.setObject({{1,2,3}, {4,5,6}});
		helper.addPoints(0, points0);
		helper.addPoints(2, points2);
		helper.next();

		helper.setObject({{1,2,3}, {4,5,6}});
		helper.addPoints(0, points0);
		helper.addPoints(1, points1);
		helper.addPoints(2, points2);
		helper.next();

		auto points = helper.getPoints({0, 1, 2}, 0);

		REQUIRE(points.size() == 3); // three cameras
		REQUIRE(helper.getPointsCount() == 4); // next called twice

		for (int i = 0; i < npoints; i++) {
			REQUIRE(points.at(0).at(i) == points0[i]);
			REQUIRE(points.at(1).at(i) == points1[i]);
			REQUIRE(points.at(2).at(i) == points2[i]);
		}
	}

	SECTION("One missing in first set, subset queried (double)") {
		// same as above, but one point is not added

		auto helper = ftl::calibration::CalibrationPoints<double>();
		int npoints = 2;
		std::vector<cv::Point2d> points0 = {{0, 0}, {0, 1}};
		std::vector<cv::Point2d> points1 = {{0, 2}, {0, 3}};
		std::vector<cv::Point2d> points2 = {{0, 4}, {0, 5}};

		helper.setObject({{1,2,3}, {4,5,6}});
		helper.addPoints(0, points0);
		helper.addPoints(2, points2);
		helper.next();

		helper.setObject({{1,2,3}, {4,5,6}});
		helper.addPoints(0, points0);
		helper.addPoints(1, points1);
		helper.addPoints(2, points2);
		helper.next();

		auto points = helper.getPoints({0, 2}, 0);

		REQUIRE(points.size() == 2);
		REQUIRE(helper.getPointsCount() == 4);

		for (int i = 0; i < npoints; i++) {
			REQUIRE(points.at(0).at(i) == points0[i]);
			REQUIRE(points.at(1).at(i) == points2[i]);
		}
	}

	SECTION("One missing in first set, subset queried in reverse order (double)") {
		// same as above, getPoints({2, 0}) instead of getPoints({0, 2})

		auto helper = ftl::calibration::CalibrationPoints<double>();
		int npoints = 2;
		std::vector<cv::Point2d> points0 = {{0, 0}, {0, 1}};
		std::vector<cv::Point2d> points1 = {{0, 2}, {0, 3}};
		std::vector<cv::Point2d> points2 = {{0, 4}, {0, 5}};

		helper.setObject({{1,2,3}, {4,5,6}});
		helper.addPoints(0, points0);
		helper.addPoints(2, points2);
		helper.next();

		helper.setObject({{7,8,9}, {10,11,12}});
		helper.addPoints(0, points0);
		helper.addPoints(1, points1);
		helper.addPoints(2, points2);
		helper.next();

		auto points = helper.getPoints({2, 0}, 0);

		REQUIRE(points.size() == 2);
		REQUIRE(helper.getPointsCount() == 4);

		for (int i = 0; i < npoints; i++) {
			REQUIRE(points.at(0).at(i) == points2[i]);
			REQUIRE(points.at(1).at(i) == points0[i]);
		}
	}
}
