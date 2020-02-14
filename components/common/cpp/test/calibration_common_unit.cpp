#include "catch.hpp"
#include "ftl/calibration.hpp"

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
