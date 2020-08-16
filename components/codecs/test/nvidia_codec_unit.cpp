#include "catch.hpp"
#include <ftl/codecs/nvidia_encoder.hpp>
#include <ftl/codecs/nvidia_decoder.hpp>
#include <ftl/codecs/hevc.hpp>
#include <ftl/threads.hpp>

#include <opencv2/cudaarithm.hpp>

using ftl::codecs::definition_t;
using ftl::codecs::codec_t;
using ftl::codecs::format_t;

ctpl::thread_pool ftl::pool(4);

namespace ftl {
	bool running = true;

	namespace codecs {
	namespace internal {
	
	void init_encoders() {}

	}
	}
}


TEST_CASE( "NvidiaEncoder::encode() - A valid colour image" ) {
	ftl::codecs::NvidiaEncoder encoder(definition_t::HD1080, definition_t::SD480);
	cv::cuda::GpuMat m(cv::Size(1920,1080), CV_8UC4, cv::Scalar(0,0,0,0));

	ftl::codecs::Packet pkt;
	pkt.codec = codec_t::Any;
	pkt.bitrate = 255;
	pkt.flags = 0;
	pkt.frame_count = 1;

	SECTION("auto codec and definition, single frame") {
		bool r = encoder.encode(m, pkt);

		REQUIRE( r );
		REQUIRE( pkt.codec == codec_t::HEVC );
		REQUIRE( pkt.flags == ftl::codecs::kFlagFlipRGB );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( ftl::codecs::hevc::validNAL(pkt.data.data(), pkt.data.size()) );
	}

	SECTION("auto codec and definition, single frame, 1 bitrate") {
		pkt.bitrate = 1;

		bool r = encoder.encode(m, pkt);

		REQUIRE( r );
		REQUIRE( pkt.bitrate == 1 );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( ftl::codecs::hevc::validNAL(pkt.data.data(), pkt.data.size()) );
	}

	SECTION("invalid frame count of 0") {
		pkt.frame_count = 0;

		bool r = encoder.encode(m, pkt);

		REQUIRE( !r );
		REQUIRE( pkt.data.size() == 0 );
	}

	SECTION("invalid float flag") {
		pkt.flags = ftl::codecs::kFlagFloat;

		bool r = encoder.encode(m, pkt);

		REQUIRE( r );
		REQUIRE( pkt.flags == ftl::codecs::kFlagFlipRGB );
		REQUIRE( pkt.data.size() != 0 );
	}

	SECTION("invalid codec") {
		pkt.codec = codec_t::JPG;

		bool r = encoder.encode(m, pkt);

		REQUIRE( !r );
		REQUIRE( pkt.codec == codec_t::Invalid );
		REQUIRE( pkt.data.size() == 0 );
	}
}

TEST_CASE( "NvidiaEncoder::encode() - A tiled colour image" ) {
	ftl::codecs::NvidiaEncoder encoder(definition_t::HD1080, definition_t::SD480);
	cv::cuda::GpuMat m(cv::Size(2560,720), CV_8UC4, cv::Scalar(0,0,0,0));

	SECTION("auto codec and definition, 2x1 frames") {
		ftl::codecs::Packet pkt;
		pkt.codec = codec_t::Any;
		pkt.bitrate = 255;
		pkt.flags = 0;
		pkt.frame_count = 2;

		bool r = encoder.encode(m, pkt);

		REQUIRE( r );
		REQUIRE( pkt.codec == codec_t::HEVC );
		REQUIRE( pkt.flags == ftl::codecs::kFlagFlipRGB );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( ftl::codecs::hevc::validNAL(pkt.data.data(), pkt.data.size()) );
	}
}

TEST_CASE( "NvidiaEncoder::encode() - A valid lossless float image" ) {
	ftl::codecs::NvidiaEncoder encoder(definition_t::HD1080, definition_t::SD480);
	cv::cuda::GpuMat m(cv::Size(1280,720), CV_32F, cv::Scalar(0.0f));

	SECTION("auto codec and definition, single frame") {
		ftl::codecs::Packet pkt;
		pkt.codec = codec_t::HEVC_LOSSLESS;
		pkt.bitrate = 255;
		pkt.flags = 0;
		pkt.frame_count = 1;

		bool r = encoder.encode(m, pkt);

		REQUIRE( r );
		REQUIRE( pkt.codec == codec_t::HEVC_LOSSLESS );
		REQUIRE( pkt.flags == ftl::codecs::kFlagFloat );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( ftl::codecs::hevc::validNAL(pkt.data.data(), pkt.data.size()) );
	}

	SECTION("invalid lossy flag") {
		ftl::codecs::Packet pkt;
		pkt.codec = codec_t::HEVC_LOSSLESS;
		pkt.bitrate = 255;
		pkt.flags = ftl::codecs::kFlagMappedDepth;
		pkt.frame_count = 1;

		bool r = encoder.encode(m, pkt);

		REQUIRE( r );
		REQUIRE( pkt.flags == ftl::codecs::kFlagFloat );
		REQUIRE( pkt.data.size() != 0 );
	}
}

TEST_CASE( "NvidiaEncoder::encode() - A valid lossy float image" ) {
	ftl::codecs::NvidiaEncoder encoder(definition_t::HD1080, definition_t::SD480);
	cv::cuda::GpuMat m(cv::Size(1280,720), CV_32F, cv::Scalar(0.0f));

	SECTION("auto codec and definition, single frame") {
		ftl::codecs::Packet pkt;
		pkt.codec = codec_t::Any;
		pkt.bitrate = 255;
		pkt.flags = 0;
		pkt.frame_count = 1;

		bool r = encoder.encode(m, pkt);

		REQUIRE( r );
		REQUIRE( pkt.codec == codec_t::HEVC );
		REQUIRE( pkt.flags == (ftl::codecs::kFlagFloat | ftl::codecs::kFlagMappedDepth) );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( ftl::codecs::hevc::validNAL(pkt.data.data(), pkt.data.size()) );
	}
}

TEST_CASE( "NvidiaEncoder::encode() - A tiled lossy float image" ) {
	ftl::codecs::NvidiaEncoder encoder(definition_t::HD1080, definition_t::SD480);
	cv::cuda::GpuMat m(cv::Size(2560,720), CV_32F, cv::Scalar(0));

	SECTION("auto codec and definition, 2x1 frame") {
		ftl::codecs::Packet pkt;
		pkt.codec = codec_t::Any;
		pkt.bitrate = 255;
		pkt.flags = ftl::codecs::kFlagFloat & ftl::codecs::kFlagMappedDepth;
		pkt.frame_count = 2;

		bool r = encoder.encode(m, pkt);

		REQUIRE( r );
		REQUIRE( pkt.codec == codec_t::HEVC );
		REQUIRE( pkt.flags == (ftl::codecs::kFlagFloat | ftl::codecs::kFlagMappedDepth) );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( ftl::codecs::hevc::validNAL(pkt.data.data(), pkt.data.size()) );
	}
}

TEST_CASE( "NvidiaEncoder::encode() - A large tiled lossy float image" ) {
	ftl::codecs::NvidiaEncoder encoder(definition_t::HD1080, definition_t::SD480);
	cv::cuda::GpuMat m(cv::Size(5120,1440), CV_32F, cv::Scalar(0));

	SECTION("auto codec and definition, 4x2 frame") {
		ftl::codecs::Packet pkt;
		pkt.codec = codec_t::Any;
		pkt.bitrate = 128;
		pkt.flags = 0;
		pkt.frame_count = 7;

		bool r = encoder.encode(m, pkt);

		REQUIRE( r );
		REQUIRE( pkt.codec == codec_t::HEVC );
		REQUIRE( pkt.flags == (ftl::codecs::kFlagFloat | ftl::codecs::kFlagMappedDepth) );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( ftl::codecs::hevc::validNAL(pkt.data.data(), pkt.data.size()) );
	}
}

TEST_CASE( "NvidiaDecoder::decode() - A colour test image" ) {
	ftl::codecs::NvidiaEncoder encoder(definition_t::HD1080, definition_t::SD480);
	ftl::codecs::NvidiaDecoder decoder;

	cv::cuda::GpuMat in;
	cv::cuda::GpuMat out;

	//SECTION("FHD in and out, FHD encoding") {
		in = cv::cuda::GpuMat(cv::Size(1920,1080), CV_8UC4, cv::Scalar(255,0,0,0));
		out = cv::cuda::GpuMat(cv::Size(1920,1080), CV_8UC4, cv::Scalar(0,0,0,0));

		ftl::codecs::Packet pkt;
		pkt.codec = codec_t::Any;
		pkt.bitrate = 255;
		pkt.frame_count = 1;
		pkt.flags = 0;
		bool r = encoder.encode(in, pkt);

		REQUIRE( r );
		REQUIRE( decoder.decode(pkt, out) );
		REQUIRE( (out.cols == 1920) );
		REQUIRE( (out.type() == CV_8UC4) );
	//}

	REQUIRE( (cv::cuda::sum(out) != cv::Scalar(0,0,0)) );
}

TEST_CASE( "NvidiaDecoder::decode() - A tiled colour image" ) {
	ftl::codecs::NvidiaEncoder encoder(definition_t::HD1080, definition_t::SD480);
	ftl::codecs::NvidiaDecoder decoder;

	cv::cuda::GpuMat in;
	cv::cuda::GpuMat out;

	//SECTION("FHD in and out, FHD encoding") {
		in = cv::cuda::GpuMat(cv::Size(2560,720), CV_8UC4, cv::Scalar(255,0,0,0));
		out = cv::cuda::GpuMat(cv::Size(2560,720), CV_8UC4, cv::Scalar(0,0,0,0));

		ftl::codecs::Packet pkt;
		pkt.codec = codec_t::Any;
		pkt.bitrate = 255;
		pkt.frame_count = 2;
		pkt.flags = 0;
		bool r = encoder.encode(in, pkt);

		REQUIRE( r );
		REQUIRE( decoder.decode(pkt, out) );
		REQUIRE( (out.cols == 2560) );
		REQUIRE( (out.type() == CV_8UC4) );
	//}

	REQUIRE( (cv::cuda::sum(out) != cv::Scalar(0,0,0)) );
}

TEST_CASE( "NvidiaDecoder::decode() - A lossless depth image" ) {
	ftl::codecs::NvidiaEncoder encoder(definition_t::HD1080, definition_t::SD480);
	ftl::codecs::NvidiaDecoder decoder;

	cv::cuda::GpuMat in;
	cv::cuda::GpuMat out;

	//SECTION("FHD in and out, FHD encoding") {
		in = cv::cuda::GpuMat(cv::Size(1280,720), CV_32F, cv::Scalar(10.0f));
		out = cv::cuda::GpuMat(cv::Size(1280,720), CV_32F, cv::Scalar(0.0f));

		ftl::codecs::Packet pkt;
		pkt.codec = codec_t::HEVC_LOSSLESS;
		pkt.bitrate = 255;
		pkt.frame_count = 1;
		pkt.flags = 0;
		bool r = encoder.encode(in, pkt);

		REQUIRE( r );
		REQUIRE( decoder.decode(pkt, out) );
	//}

	REQUIRE( (cv::cuda::sum(out) != cv::Scalar(0)) );
}

TEST_CASE( "NvidiaDecoder::decode() - A lossy depth image" ) {
	ftl::codecs::NvidiaEncoder encoder(definition_t::HD1080, definition_t::SD480);
	ftl::codecs::NvidiaDecoder decoder;

	cv::cuda::GpuMat in;
	cv::cuda::GpuMat out;

	//SECTION("FHD in and out, FHD encoding") {
		in = cv::cuda::GpuMat(cv::Size(1280,720), CV_32F, cv::Scalar(10.0f));
		out = cv::cuda::GpuMat(cv::Size(1280,720), CV_32F, cv::Scalar(0));

		ftl::codecs::Packet pkt;
		pkt.codec = codec_t::Any;
		pkt.bitrate = 255;
		pkt.frame_count = 1;
		pkt.flags = 0;
		bool r = encoder.encode(in, pkt);

		REQUIRE( r );
		REQUIRE( decoder.decode(pkt, out) );
	//}

	REQUIRE( (cv::cuda::sum(out) != cv::Scalar(0)) );
}

TEST_CASE( "NvidiaDecoder::decode() - corrupted packet" ) {
	ftl::codecs::NvidiaEncoder encoder(definition_t::HD1080, definition_t::SD480);
	ftl::codecs::NvidiaDecoder decoder;

	cv::cuda::GpuMat in;
	cv::cuda::GpuMat out;

	SECTION("Corrupted but supported codec") {
		in = cv::cuda::GpuMat(cv::Size(2560,720), CV_8UC4, cv::Scalar(255,0,0,0));
		out = cv::cuda::GpuMat(cv::Size(2560,720), CV_8UC4, cv::Scalar(0,0,0,0));

		ftl::codecs::Packet pkt;
		pkt.codec = codec_t::Any;
		pkt.bitrate = 255;
		pkt.frame_count = 2;
		pkt.flags = 0;
		bool r = encoder.encode(in, pkt);

		pkt.codec = codec_t::H264;

		REQUIRE( r );
		REQUIRE( !decoder.decode(pkt, out) );
	}

	SECTION("Corrupted and unsupported codec") {
		in = cv::cuda::GpuMat(cv::Size(2560,720), CV_8UC4, cv::Scalar(255,0,0,0));
		out = cv::cuda::GpuMat(cv::Size(2560,720), CV_8UC4, cv::Scalar(0,0,0,0));

		ftl::codecs::Packet pkt;
		pkt.codec = codec_t::Any;
		pkt.bitrate = 255;
		pkt.frame_count = 2;
		pkt.flags = 0;
		bool r = encoder.encode(in, pkt);

		pkt.codec = codec_t::JPG;

		REQUIRE( r );
		REQUIRE( !decoder.decode(pkt, out) );
	}

	SECTION("Corrupted float flag") {
		in = cv::cuda::GpuMat(cv::Size(2560,720), CV_8UC4, cv::Scalar(255,0,0,0));
		out = cv::cuda::GpuMat(cv::Size(2560,720), CV_8UC4, cv::Scalar(0,0,0,0));

		ftl::codecs::Packet pkt;
		pkt.codec = codec_t::Any;
		pkt.bitrate = 255;
		pkt.frame_count = 2;
		pkt.flags = 0;
		bool r = encoder.encode(in, pkt);

		pkt.flags = ftl::codecs::kFlagFloat;

		REQUIRE( r );
		REQUIRE( decoder.decode(pkt, out) );
		REQUIRE( out.type() == CV_32F );
	}

	SECTION("Corrupted float mapped flags") {
		in = cv::cuda::GpuMat(cv::Size(1280,720), CV_32F, cv::Scalar(10.0f));
		out = cv::cuda::GpuMat(cv::Size(1280,720), CV_32F, cv::Scalar(0));

		ftl::codecs::Packet pkt;
		pkt.codec = codec_t::HEVC_LOSSLESS;
		pkt.bitrate = 255;
		pkt.frame_count = 1;
		pkt.flags = ftl::codecs::kFlagFloat;
		bool r = encoder.encode(in, pkt);

		//pkt.codec = codec_t::HEVC;
		pkt.flags = ftl::codecs::kFlagFloat | ftl::codecs::kFlagMappedDepth;

		REQUIRE( r );
		REQUIRE( decoder.decode(pkt, out) );
	}

	SECTION("Missing float flag - lossless") {
		in = cv::cuda::GpuMat(cv::Size(1280,720), CV_32F, cv::Scalar(255));
		out = cv::cuda::GpuMat(cv::Size(1280,720), CV_32F, cv::Scalar(0));

		ftl::codecs::Packet pkt;
		pkt.codec = codec_t::HEVC_LOSSLESS;
		pkt.bitrate = 255;
		pkt.frame_count = 1;
		pkt.flags = ftl::codecs::kFlagFloat;
		bool r = encoder.encode(in, pkt);

		pkt.flags = 0;

		REQUIRE( r );
		REQUIRE( decoder.decode(pkt, out) );
		REQUIRE( out.type() == CV_8UC4 );
		REQUIRE( out.cols == 2*in.cols );
	}

	SECTION("Missing data") {
		in = cv::cuda::GpuMat(cv::Size(1280,720), CV_32F, cv::Scalar(255));
		out = cv::cuda::GpuMat(cv::Size(1280,720), CV_32F, cv::Scalar(0));

		ftl::codecs::Packet pkt;
		pkt.codec = codec_t::Any;
		pkt.bitrate = 255;
		pkt.frame_count = 1;
		pkt.flags = 0;
		bool r = encoder.encode(in, pkt);

		pkt.data.resize(0);

		REQUIRE( r );
		REQUIRE( !decoder.decode(pkt, out) );
	}
}
