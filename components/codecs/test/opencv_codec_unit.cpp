#include "catch.hpp"
#include <ftl/codecs/opencv_encoder.hpp>
#include <ftl/codecs/opencv_decoder.hpp>
#include <ftl/threads.hpp>

using ftl::codecs::format_t;
using ftl::codecs::definition_t;
using ftl::codecs::codec_t;

ctpl::thread_pool ftl::pool(4);

namespace ftl {
	bool running = true;

	namespace codecs {
	namespace internal {
	
	void init_encoders() {}

	}
	}
}
/*
TEST_CASE( "OpenCVEncoder::encode() - A colour test image at preset 0" ) {
	ftl::codecs::OpenCVEncoder encoder(definition_t::HD1080, definition_t::SD480);

	int block_total = 0;
	std::atomic<int> block_count = 0;

	const CodecPreset &preset = ftl::codecs::getPreset(ftl::codecs::kPreset4);
	cv::cuda::GpuMat m(cv::Size(ftl::codecs::getWidth(preset.res),
								ftl::codecs::getHeight(preset.res)),
						CV_8UC3, cv::Scalar(0,0,0));

	std::mutex mtx;

	bool r = encoder.encode(m, ftl::codecs::kPreset4, [&mtx, &block_total, &block_count, preset, m](const ftl::codecs::Packet &pkt) {
		std::unique_lock<std::mutex> lk(mtx);
		REQUIRE( pkt.codec == codec_t::JPG );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( pkt.definition == preset.res );

		block_total = pkt.block_total;
		block_count++;

		cv::Mat d = cv::imdecode(pkt.data, cv::IMREAD_UNCHANGED);
		REQUIRE( !d.empty() );
		REQUIRE( d.cols * d.rows * pkt.block_total == m.cols * m.rows );
	});

	REQUIRE( r );
	REQUIRE( block_count == block_total );
}

TEST_CASE( "OpenCVEncoder::encode() - A depth test image at preset 0" ) {
	ftl::codecs::OpenCVEncoder encoder(definition_t::HD1080, definition_t::SD480);
	cv::cuda::GpuMat m(cv::Size(1024,576), CV_32F, cv::Scalar(0.0f));

	int block_total = 0;
	std::atomic<int> block_count = 0;

	const CodecPreset &preset = ftl::codecs::getPreset(ftl::codecs::kPreset4);

	std::mutex mtx;

	bool r = encoder.encode(m, ftl::codecs::kPreset4, [&mtx, &block_total, &block_count, preset](const ftl::codecs::Packet &pkt) {
		std::unique_lock<std::mutex> lk(mtx);
		REQUIRE( pkt.codec == codec_t::PNG );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( pkt.definition == preset.res );

		block_total = pkt.block_total;
		block_count++;

		cv::Mat d = cv::imdecode(pkt.data, cv::IMREAD_UNCHANGED);
		REQUIRE( !d.empty() );
	});

	REQUIRE( r );
	REQUIRE( block_count == block_total );
}
*/
TEST_CASE( "OpenCVDecoder::decode() - A colour test image no resolution change" ) {
	ftl::codecs::OpenCVEncoder encoder(definition_t::HD1080, definition_t::SD480);
	ftl::codecs::OpenCVDecoder decoder;
	cv::cuda::GpuMat in(cv::Size(1024,576), CV_8UC4, cv::Scalar(255,0,0,0));
	cv::cuda::GpuMat out(cv::Size(1024,576), CV_8UC4, cv::Scalar(0,0,0,0));

	ftl::codecs::Packet pkt;
	pkt.codec = codec_t::Any;
	pkt.definition = definition_t::Any;
	pkt.bitrate = 255;
	pkt.flags = 0;
	pkt.frame_count = 1;
	bool r = encoder.encode(in, pkt);

	REQUIRE( r );
	REQUIRE( decoder.decode(pkt, out) );

	REQUIRE( (cv::cuda::sum(out) != cv::Scalar(0,0,0,0)) );
}
