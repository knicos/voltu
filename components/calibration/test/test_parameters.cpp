#include "catch.hpp"
#include <ftl/calibration/parameters.hpp>
#include <ftl/calibration/structures.hpp>

using cv::Size;
using cv::Mat;

TEST_CASE("Calibration values", "") {
	SECTION("Valid distortion parameters (k1, k2, k3, k4)") {
		Mat D(1, 5, CV_64FC1);
		D.at<double>(0) = 0.0;
		D.at<double>(1) = 0.0;
		D.at<double>(2) = 0.0;
		D.at<double>(3) = 0.0;

		REQUIRE(ftl::calibration::validate::distortionCoefficients(D, Size(1920, 1080)));

		D.at<double>(0) = 1.0;
		D.at<double>(1) = 1.0;
		REQUIRE(ftl::calibration::validate::distortionCoefficients(D, Size(1920, 1080)));

		D.at<double>(0) =  0.01512461889185869;
		D.at<double>(1) = -0.1207895096066378;
		D.at<double>(4) =  0.1582571415357494;
		REQUIRE(ftl::calibration::validate::distortionCoefficients(D, Size(1920, 1080)));
	}

	SECTION("Invalid distortion parameters (k1, k2, k3, k4)") {
		Mat D(1, 4, CV_64FC1);
		D.at<double>(0) = NAN;
		D.at<double>(1) = 0.0;
		D.at<double>(2) = 0.0;
		D.at<double>(3) = 0.0;

		REQUIRE(!ftl::calibration::validate::distortionCoefficients(D, Size(1920, 1080)));

		D.at<double>(0) = 1.0;
		D.at<double>(1) = -1.0;
		REQUIRE(!ftl::calibration::validate::distortionCoefficients(D, Size(1920, 1080)));
	}
}

TEST_CASE("Test reading/writing file") {
	using ftl::calibration::CalibrationData;
	using ftl::codecs::Channel;

	CalibrationData::Calibration calib;
	CalibrationData calib_read;
	CalibrationData data;

	calib.intrinsic.resolution = {1, 1};
	calib.intrinsic.fx = 1.0;
	calib.intrinsic.fy = 1.0;
	calib.intrinsic.cx = 0.5;
	calib.intrinsic.cy = 0.5;
	data.get(Channel::Left) = calib;

	data.writeFile("/tmp/calib.yml");
	calib_read = CalibrationData::readFile("/tmp/calib.yml");
	REQUIRE(calib_read.hasCalibration(Channel::Left));

	data.writeFile("/tmp/calib.json");
	calib_read = CalibrationData::readFile("/tmp/calib.json");
	REQUIRE(calib_read.hasCalibration(Channel::Left));

	data.writeFile("/tmp/calib.xml");
	calib_read = CalibrationData::readFile("/tmp/calib.xml");
	REQUIRE(calib_read.hasCalibration(Channel::Left));
}
