#include "catch.hpp"
#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

#include <iostream>

#include <opencv2/core.hpp>
#include <ftl/utility/msgpack.hpp>

using cv::Mat;
using cv::Size;
using cv::Rect;

template <typename T>
std::string msgpack_pack(T v) {
	std::stringstream buffer;
	msgpack::pack(buffer, v);
	buffer.seekg(0);
	return std::string(buffer.str());
}

Mat msgpack_unpack_mat(std::string str) {
	msgpack::object_handle oh = msgpack::unpack(str.data(), str.size());
	msgpack::object obj = oh.get();
	Mat M;
	return obj.convert<Mat>(M);
}

TEST_CASE( "msgpack cv::Mat" ) {
	SECTION( "Mat::ones(Size(5, 5), CV_64FC1)" ) {
		Mat A = Mat::ones(Size(5, 5), CV_64FC1);
		Mat B = msgpack_unpack_mat(msgpack_pack(A));

		REQUIRE(A.size() == B.size());
		REQUIRE(A.type() == B.type());
		REQUIRE(cv::countNonZero(A != B) == 0);
	}

	SECTION( "Mat::ones(Size(1, 5), CV_8UC3)" ) {
		Mat A = Mat::ones(Size(1, 5), CV_8UC3);
		Mat B = msgpack_unpack_mat(msgpack_pack(A));
		
		REQUIRE(A.size() == B.size());
		REQUIRE(A.type() == B.type());
		REQUIRE(cv::countNonZero(A != B) == 0);
	}

	SECTION ( "Mat 10x10 CV_64FC1 with random values [-1000, 1000]" ) {
		Mat A(Size(10, 10), CV_64FC1);
		cv::randu(A, -1000, 1000);
		Mat B = msgpack_unpack_mat(msgpack_pack(A));
		
		REQUIRE(A.size() == B.size());
		REQUIRE(A.type() == B.type());
		REQUIRE(cv::countNonZero(A != B) == 0);
	}

	SECTION( "Test object_with_zone created from Mat with random values" ) {
		Mat A(Size(10, 10), CV_64FC1);
		cv::randu(A, -1000, 1000);

		msgpack::zone z;
		auto obj = msgpack::object(A, z);
		
		Mat B = msgpack_unpack_mat(msgpack_pack(obj));
		
		REQUIRE(A.size() == B.size());
		REQUIRE(A.type() == B.type());
		REQUIRE(cv::countNonZero(A != B) == 0);
	}

	SECTION( "Non-continuous Mat" ) {
		try {
			Mat A = Mat::ones(Size(10, 10), CV_8UC1);
			A = A(Rect(2, 2, 3,3));
			A.setTo(0);

			Mat B = msgpack_unpack_mat(msgpack_pack(A));
		
			REQUIRE(A.size() == B.size());
			REQUIRE(A.type() == B.type());
			REQUIRE(cv::countNonZero(A != B) == 0);
		}
		catch (msgpack::type_error) {
			// if not supported, throws exception
		}
	}
}
