#include "catch.hpp"
#include <ftl/codecs/nvpipe_encoder.hpp>
#include <ftl/codecs/nvpipe_decoder.hpp>
#include <ftl/threads.hpp>

using ftl::codecs::CodecPreset;
using ftl::codecs::preset_t;
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

TEST_CASE( "NvPipeEncoder::encode() - A colour test image at preset 0" ) {
	ftl::codecs::NvPipeEncoder encoder(definition_t::HD1080, definition_t::SD480);
	cv::Mat m(cv::Size(1920,1080), CV_8UC3, cv::Scalar(0,0,0));

	int block_total = 0;
	std::atomic<int> block_count = 0;

	const CodecPreset &preset = ftl::codecs::getPreset(ftl::codecs::kPreset0);

	bool r = encoder.encode(m, ftl::codecs::kPreset0, [&block_total, &block_count, preset, m](const ftl::codecs::Packet &pkt) {
		REQUIRE( pkt.codec == codec_t::HEVC );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( pkt.definition == preset.colour_res );

		block_total = pkt.block_total;
		block_count++;
	});

	REQUIRE( r );
	REQUIRE( block_count == block_total );
}

TEST_CASE( "NvPipeEncoder::encode() - A depth test image at preset 0" ) {
	ftl::codecs::NvPipeEncoder encoder(definition_t::HD1080, definition_t::SD480);
	cv::Mat m(cv::Size(1920,1080), CV_32F, cv::Scalar(0.0f));

	int block_total = 0;
	std::atomic<int> block_count = 0;

	const CodecPreset &preset = ftl::codecs::getPreset(ftl::codecs::kPreset0);

	bool r = encoder.encode(m, ftl::codecs::kPreset0, [&block_total, &block_count, preset](const ftl::codecs::Packet &pkt) {
		REQUIRE( pkt.codec == codec_t::HEVC );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( pkt.definition == preset.depth_res );

		block_total = pkt.block_total;
		block_count++;
	});

	REQUIRE( r );
	REQUIRE( block_count == block_total );
}

TEST_CASE( "NvPipeDecoder::decode() - A colour test image" ) {
	ftl::codecs::NvPipeEncoder encoder(definition_t::HD1080, definition_t::SD480);
	ftl::codecs::NvPipeDecoder decoder;

	cv::Mat in;
	cv::Mat out;
	bool r = false;

	SECTION("FHD in and out, FHD encoding") {
		in = cv::Mat(cv::Size(1920,1080), CV_8UC3, cv::Scalar(255,0,0));
		out = cv::Mat(cv::Size(1920,1080), CV_8UC3, cv::Scalar(0,0,0));

		r = encoder.encode(in, ftl::codecs::kPreset0, [&out,&decoder](const ftl::codecs::Packet &pkt) {
			REQUIRE( decoder.decode(pkt, out) );
		});
	}

	SECTION("Full HD in, 720 out, FHD encoding") {
		in = cv::Mat(cv::Size(1920,1080), CV_8UC3, cv::Scalar(255,0,0));
		out = cv::Mat(cv::Size(1280,720), CV_8UC3, cv::Scalar(0,0,0));

		r = encoder.encode(in, ftl::codecs::kPreset0, [&out,&decoder](const ftl::codecs::Packet &pkt) {
			REQUIRE( decoder.decode(pkt, out) );
		});

		REQUIRE( (out.rows == 720) );
	}

	SECTION("HHD in, FHD out, FHD encoding") {
		in = cv::Mat(cv::Size(1280,720), CV_8UC3, cv::Scalar(255,0,0));
		out = cv::Mat(cv::Size(1920,1080), CV_8UC3, cv::Scalar(0,0,0));

		r = encoder.encode(in, ftl::codecs::kPreset0, [&out,&decoder](const ftl::codecs::Packet &pkt) {
			REQUIRE( decoder.decode(pkt, out) );
		});

		REQUIRE( (out.rows == 1080) );
	}

	SECTION("FHD in, HHD out, SD encoding") {
		in = cv::Mat(cv::Size(1920,1080), CV_8UC3, cv::Scalar(255,0,0));
		out = cv::Mat(cv::Size(1280,720), CV_8UC3, cv::Scalar(0,0,0));

		r = encoder.encode(in, ftl::codecs::kPreset4, [&out,&decoder](const ftl::codecs::Packet &pkt) {
			REQUIRE( decoder.decode(pkt, out) );
		});

		REQUIRE( (out.rows == 720) );
	}

	REQUIRE( r );
	REQUIRE( (cv::sum(out) != cv::Scalar(0,0,0)) );
}
