#include "catch.hpp"
#include <ftl/rgbd/frame.hpp>

using ftl::rgbd::Frame;
using ftl::codecs::Channel;
using ftl::codecs::Channels;
using ftl::rgbd::Format;

TEST_CASE("Frame::create() cpu mat", "") {
	SECTION("in empty channel with format") {
		Frame f;
		auto &m = f.create<cv::Mat>(Channel::Colour, Format<float4>(200,200));

		REQUIRE( m.type() == CV_32FC4 );
		REQUIRE( m.cols == 200 );
		REQUIRE( m.rows == 200 );
	}

	SECTION("in non-empty channel with format") {
		Frame f;
		f.create<cv::Mat>(Channel::Colour, Format<float>(200,100));
		auto &m = f.create<cv::Mat>(Channel::Colour, Format<float4>(200,200));

		REQUIRE( m.type() == CV_32FC4 );
		REQUIRE( m.cols == 200 );
		REQUIRE( m.rows == 200 );
	}
}

TEST_CASE("Frame::get()", "") {
	SECTION("get a non-existant host channel") {
		Frame f;
		bool hadexception = false;

		try {
			f.get<cv::Mat>(Channel::Colour);
		} catch (ftl::exception &e) {
			hadexception = true;
		}

		REQUIRE( hadexception );
	}

	SECTION("get a non-existant gpu channel") {
		Frame f;
		bool hadexception = false;

		try {
			f.get<cv::cuda::GpuMat>(Channel::Colour);
		} catch (ftl::exception &e) {
			hadexception = true;
		}

		REQUIRE( hadexception );
	}

	SECTION("get a valid host channel") {
		Frame f;
		bool hadexception = false;

		try {
			f.create<cv::Mat>(Channel::Colour, Format<uchar3>(1024,1024));
			auto &m = f.get<cv::Mat>(Channel::Colour);

			REQUIRE( m.type() == CV_8UC3 );
			REQUIRE( m.cols == 1024 );
			REQUIRE( m.rows == 1024 );
		} catch (ftl::exception &e) {
			hadexception = true;
		}

		REQUIRE( !hadexception );
	}

	SECTION("get a valid gpu channel") {
		Frame f;
		bool hadexception = false;

		try {
			f.create<cv::cuda::GpuMat>(Channel::Colour, Format<uchar3>(1024,1024));
			auto &m = f.get<cv::cuda::GpuMat>(Channel::Colour);

			REQUIRE( m.type() == CV_8UC3 );
			REQUIRE( m.cols == 1024 );
			REQUIRE( m.rows == 1024 );
		} catch (ftl::exception &e) {
			hadexception = true;
		}

		REQUIRE( !hadexception );
	}

	SECTION("get a cpu mat from gpu channel") {
		Frame f;
		bool hadexception = false;

		try {
			f.create<cv::cuda::GpuMat>(Channel::Colour, Format<uchar3>(1024,1024));
			REQUIRE( f.isGPU(Channel::Colour) );

			auto &m = f.get<cv::Mat>(Channel::Colour);

			REQUIRE( f.isCPU(Channel::Colour) );
			REQUIRE( m.type() == CV_8UC3 );
			REQUIRE( m.cols == 1024 );
			REQUIRE( m.rows == 1024 );
		} catch (ftl::exception &e) {
			hadexception = true;
		}

		REQUIRE( !hadexception );
	}

	SECTION("get a gpu mat from cpu channel") {
		Frame f;
		bool hadexception = false;

		try {
			f.create<cv::Mat>(Channel::Colour, Format<uchar3>(1024,1024));
			REQUIRE( f.isCPU(Channel::Colour) );
			
			auto &m = f.get<cv::cuda::GpuMat>(Channel::Colour);

			REQUIRE( f.isGPU(Channel::Colour) );
			REQUIRE( m.type() == CV_8UC3 );
			REQUIRE( m.cols == 1024 );
			REQUIRE( m.rows == 1024 );
		} catch (ftl::exception &e) {
			hadexception = true;
		}

		REQUIRE( !hadexception );
	}
}

TEST_CASE("Frame::createTexture()", "") {
	SECTION("Missing format and no existing mat") {
		Frame f;
		bool hadexception = false;

		try {
			f.createTexture<float>(Channel::Depth);
		} catch (ftl::exception &e) {
			hadexception = true;
		}

		REQUIRE( hadexception );
	}

	SECTION("Missing format but with existing host mat") {
		Frame f;
		bool hadexception = false;

		try {
			f.create<cv::Mat>(Channel::Depth, Format<float>(100,100));
			auto &t = f.createTexture<float>(Channel::Depth);

			REQUIRE( t.width() == 100 );
			REQUIRE( t.cvType() == CV_32FC1 );
		} catch (ftl::exception &e) {
			hadexception = true;
		}

		REQUIRE( !hadexception );
	}

	SECTION("Missing format but with incorrect existing host mat") {
		Frame f;
		bool hadexception = false;

		try {
			f.create<cv::Mat>(Channel::Depth, Format<uchar4>(100,100));
			f.createTexture<float>(Channel::Depth);
		} catch (ftl::exception &e) {
			hadexception = true;
		}

		REQUIRE( hadexception );
	}

	SECTION("With format and no existing mat") {
		Frame f;
		bool hadexception = false;

		try {
			auto &t = f.createTexture<float>(Channel::Depth, Format<float>(1024,1024));
			REQUIRE( t.cvType() == CV_32FC1 );
			REQUIRE( t.cudaTexture() > 0 );
			REQUIRE( t.devicePtr() != nullptr );

			auto &m = f.get<cv::cuda::GpuMat>(Channel::Depth);
			REQUIRE( m.data == reinterpret_cast<uchar*>(t.devicePtr()) );
			REQUIRE( m.type() == CV_32FC1 );
		} catch (ftl::exception &e) {
			hadexception = true;
		}

		REQUIRE( !hadexception );
	}

	SECTION("Unchanged type is same texture object") {
		Frame f;
		bool hadexception = false;

		try {
			auto &t = f.createTexture<float>(Channel::Depth, Format<float>(1024,1024));
			REQUIRE( t.cvType() == CV_32FC1 );
			
			auto tex = t.cudaTexture();
			float *ptr = t.devicePtr();

			REQUIRE( ptr != nullptr );

			auto &t2 = f.createTexture<float>(Channel::Depth, Format<float>(1024,1024));

			REQUIRE( tex == t2.cudaTexture() );
			REQUIRE( ptr == t2.devicePtr() );
		} catch (ftl::exception &e) {
			hadexception = true;
		}

		REQUIRE( !hadexception );
	}
}

TEST_CASE("Frame::getTexture()", "") {
	SECTION("Missing texture") {
		Frame f;
		bool hadexception = false;

		try {
			f.getTexture<float>(Channel::Depth);
		} catch (ftl::exception &e) {
			hadexception = true;
		}

		REQUIRE( hadexception );
	}

	SECTION("Texture of incorrect type") {
		Frame f;
		bool hadexception = false;

		try {
			f.createTexture<uchar4>(Channel::Depth, Format<uchar4>(100,100));
			f.getTexture<float>(Channel::Depth);
		} catch (ftl::exception &e) {
			hadexception = true;
		}

		REQUIRE( hadexception );
	}

	SECTION("Valid texture get") {
		Frame f;
		bool hadexception = false;

		try {
			f.createTexture<uchar4>(Channel::Colour, Format<uchar4>(100,100));
			auto &t = f.getTexture<uchar4>(Channel::Colour);

			REQUIRE( t.cvType() == CV_8UC4 );
			REQUIRE( t.width() == 100 );
		} catch (ftl::exception &e) {
			hadexception = true;
		}

		REQUIRE( !hadexception );
	}
}

TEST_CASE("Frame::swapTo()", "") {
	SECTION("Single host channel to empty frame") {
		Frame f1;
		Frame f2;

		f1.create<cv::Mat>(Channel::Colour, Format<uchar3>(100,100));
		f1.swapTo(Channels::All(), f2);

		REQUIRE( f2.hasChannel(Channel::Colour) );
		REQUIRE( (f2.get<cv::Mat>(Channel::Colour).cols == 100) );
	}
}
