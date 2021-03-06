#include "catch.hpp"
#include <ftl/rgbd/frame.hpp>

using ftl::rgbd::Frame;
using ftl::rgbd::FrameState;
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

		REQUIRE( hadexception );
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

		REQUIRE( hadexception );
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
		f1.swapTo(Channels<0>::All(), f2);

		REQUIRE( f2.hasChannel(Channel::Colour) );
		REQUIRE( (f2.get<cv::Mat>(Channel::Colour).cols == 100) );
	}
}

TEST_CASE("Frame::setOrigin()", "") {
	SECTION("With changed pose") {
		Frame f;
		FrameState s;

		REQUIRE( !f.hasChanged(Channel::Pose) );

		s.setPose(Eigen::Matrix4d());
		f.setOrigin(&s);

		REQUIRE( f.hasChanged(Channel::Pose) );
	}

	SECTION("With stale pose") {
		Frame f;
		FrameState s;

		REQUIRE( !f.hasChanged(Channel::Pose) );

		s.setPose(Eigen::Matrix4d());
		f.setOrigin(&s);

		f.reset();
		f.setOrigin(&s);

		REQUIRE( !f.hasChanged(Channel::Pose) );
	}

	SECTION("With updated pose") {
		Frame f;
		FrameState s;

		REQUIRE( !f.hasChanged(Channel::Pose) );

		s.setPose(Eigen::Matrix4d());
		f.setOrigin(&s);

		f.reset();
		f.setOrigin(&s);

		REQUIRE( !f.hasChanged(Channel::Pose) );

		s.setPose(Eigen::Matrix4d());

		REQUIRE( !f.hasChanged(Channel::Pose) );
		REQUIRE( s.hasChanged(Channel::Pose) );
	}

	SECTION("Fail on multi set") {
		Frame f;
		FrameState s;

		s.setPose(Eigen::Matrix4d());
		f.setOrigin(&s);

		bool failed = false;
		try {
			f.setOrigin(&s);
		} catch (...) {
			failed = true;
		}

		REQUIRE( failed );
	}

	SECTION("Reset and multi set") {
		Frame f;
		FrameState s;

		s.setPose(Eigen::Matrix4d());
		f.setOrigin(&s);

		f.reset();
		f.setOrigin(&s);
	}
}

TEST_CASE("Frame::get() Pose", "") {
	SECTION("Get valid pose") {
		Frame f;
		FrameState s;

		Eigen::Matrix4d pose1;
		s.setPose(pose1);
		f.setOrigin(&s);

		REQUIRE( f.hasChannel(Channel::Pose) );
		REQUIRE( f.hasChanged(Channel::Pose) );

		auto pose2 = f.getPose();

		REQUIRE( pose1 == pose2 );
	}
}

TEST_CASE("Frame::get() Data channel", "") {
	SECTION("Get valid data") {
		Frame f;
		
		auto val_in = std::make_tuple(55,87.0f);
		decltype(val_in) val_out;

		f.create(Channel::Data, val_in);
		f.get(Channel::Data, val_out);

		REQUIRE( std::get<0>(val_in) == std::get<0>(val_out) );
		REQUIRE( std::get<1>(val_in) == std::get<1>(val_out) );
	}

	SECTION("Read from non existing channel") {
		Frame f;
		
		auto val_in = std::make_tuple(55,87.0f);
		decltype(val_in) val_out;

		//f.create(Channel::Data, val_in);

		bool except = false;
		try {
			f.get(Channel::Data, val_out);
		} catch (...) {
			except = true;
		}

		REQUIRE( except );
	}

	SECTION("Read from non data channel") {
		Frame f;
		
		auto val_in = std::make_tuple(55,87.0f);
		decltype(val_in) val_out;

		//f.create(Channel::Data, val_in);

		bool except = false;
		try {
			f.get(Channel::Colour, val_out);
		} catch (...) {
			except = true;
		}

		REQUIRE( except );
	}

	SECTION("Use non data channel") {
		Frame f;
		
		auto val_in = std::make_tuple(55,87.0f);
		decltype(val_in) val_out;

		bool except = false;
		try {
			f.create(Channel::Colour, val_in);
		} catch (...) {
			except = true;
		}

		REQUIRE( except );
	}

	SECTION("Mix types") {
		Frame f;
		
		std::string val_in = "Hello World";
		std::tuple<int,float> val_out;

		f.create(Channel::Data, val_in);

		bool except = false;
		try {
			f.get(Channel::Data, val_out);
		} catch (...) {
			except = true;
		}

		REQUIRE( except );
	}

	SECTION("Has channel after create") {
		Frame f;
		
		std::string val_in = "Hello World";

		REQUIRE( !f.hasChannel(Channel::Data) );
		f.create(Channel::Data, val_in);
		REQUIRE( f.hasChannel(Channel::Data) );
	}
}

TEST_CASE("Frame::swapTo() Data channel", "") {
	SECTION("Swap valid data") {
		Frame f1;
		Frame f2;
		
		auto val_in = std::make_tuple(55,87.0f);
		auto val_in2 = std::make_tuple(52,7.0f);
		decltype(val_in) val_out;

		f1.create(Channel::Data, val_in);

		REQUIRE( f1.hasChannel(Channel::Data) );
		REQUIRE( !f2.hasChannel(Channel::Data) );

		f1.swapTo(Channels<0>::All(), f2);

		REQUIRE( !f1.hasChannel(Channel::Data) );
		REQUIRE( f2.hasChannel(Channel::Data) );

		f1.create(Channel::Data, val_in2);
		f2.get(Channel::Data, val_out);

		REQUIRE( std::get<0>(val_in) == std::get<0>(val_out) );
		REQUIRE( std::get<1>(val_in) == std::get<1>(val_out) );
	}
}
