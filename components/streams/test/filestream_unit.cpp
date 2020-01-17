#include "catch.hpp"

#include <ftl/streams/filestream.hpp>

using ftl::stream::File;
using ftl::stream::Stream;
using ftl::config::json_t;

TEST_CASE("ftl::stream::File write and read", "[stream]") {
	json_t global = json_t{{"$id","ftl://test"}};
	ftl::config::configure(global);

	json_t cfg = json_t{
		{"$id","ftl://test/1"}
	};
	json_t cfg2 = json_t{
		{"$id","ftl://test/2"}
	};

	auto *reader = ftl::create<File>(cfg);
	REQUIRE(reader);
	auto *writer = ftl::create<File>(cfg2);
	REQUIRE(writer);

	SECTION("write read single packet") {
		writer->set("filename", "/tmp/ftl_file_stream_test.ftl");
		writer->setMode(File::Mode::Write);

		REQUIRE( writer->begin() );

		REQUIRE( writer->post({4,ftl::timer::get_time(),2,1,ftl::codecs::Channel::Confidence},{ftl::codecs::codec_t::Any, ftl::codecs::definition_t::Any, 0, 0, 0, {'f'}}) );

		writer->end();

		reader->set("filename", "/tmp/ftl_file_stream_test.ftl");

		ftl::codecs::StreamPacket tspkt = {4,0,0,1,ftl::codecs::Channel::Colour};
		REQUIRE( reader->onPacket([&tspkt](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
			tspkt = spkt;
		}) );
		REQUIRE( reader->begin(false) );

		//reader->tick();

		//REQUIRE( tspkt.timestamp == 0 );
		REQUIRE( tspkt.streamID == (uint8_t)2 );
		REQUIRE( tspkt.channel == ftl::codecs::Channel::Confidence );
	}

	SECTION("write read multiple packets at same timestamp") {
		writer->set("filename", "/tmp/ftl_file_stream_test.ftl");
		writer->setMode(File::Mode::Write);

		REQUIRE( writer->begin() );

		REQUIRE( writer->post({4,0,0,1,ftl::codecs::Channel::Confidence},{ftl::codecs::codec_t::Any, ftl::codecs::definition_t::Any, 0, 0, 0, {'f'}}) );
		REQUIRE( writer->post({4,0,1,1,ftl::codecs::Channel::Depth},{ftl::codecs::codec_t::Any, ftl::codecs::definition_t::Any, 0, 0, 0, {'f'}}) );
		REQUIRE( writer->post({4,0,2,1,ftl::codecs::Channel::Screen},{ftl::codecs::codec_t::Any, ftl::codecs::definition_t::Any, 0, 0, 0, {'f'}}) );

		writer->end();

		reader->set("filename", "/tmp/ftl_file_stream_test.ftl");

		ftl::codecs::StreamPacket tspkt = {4,0,0,1,ftl::codecs::Channel::Colour};
		int count = 0;
		REQUIRE( reader->onPacket([&tspkt,&count](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
			tspkt = spkt;
			++count;
		}) );
		REQUIRE( reader->begin(false) );

		//reader->tick();

		REQUIRE( count == 3 );
		REQUIRE( tspkt.timestamp == 0 );
		REQUIRE( tspkt.streamID == 2 );
		REQUIRE( tspkt.channel == ftl::codecs::Channel::Screen );
	}

	SECTION("write read multiple packets at different timestamps") {
		writer->set("filename", "/tmp/ftl_file_stream_test.ftl");
		writer->setMode(File::Mode::Write);

		REQUIRE( writer->begin() );

		auto time = ftl::timer::get_time();
		REQUIRE( writer->post({4,time,0,1,ftl::codecs::Channel::Confidence},{ftl::codecs::codec_t::Any, ftl::codecs::definition_t::Any, 0, 0, 0, {'f'}}) );
		REQUIRE( writer->post({4,time+ftl::timer::getInterval(),0,1,ftl::codecs::Channel::Depth},{ftl::codecs::codec_t::Any, ftl::codecs::definition_t::Any, 0, 0, 0, {'f'}}) );
		REQUIRE( writer->post({4,time+2*ftl::timer::getInterval(),0,1,ftl::codecs::Channel::Screen},{ftl::codecs::codec_t::Any, ftl::codecs::definition_t::Any, 0, 0, 0, {'f'}}) );

		writer->end();

		reader->set("filename", "/tmp/ftl_file_stream_test.ftl");

		ftl::codecs::StreamPacket tspkt = {4,0,0,1,ftl::codecs::Channel::Colour};
		int count = 0;
		REQUIRE( reader->onPacket([&tspkt,&count](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
			tspkt = spkt;
			++count;
		}) );
		REQUIRE( reader->begin(false) );

		//reader->tick();

		REQUIRE( count == 1 );
		//REQUIRE( tspkt.timestamp == 0 );
		auto itime = tspkt.timestamp;

		count = 0;
		reader->tick();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		REQUIRE( count == 1 );
		REQUIRE( tspkt.timestamp == itime+ftl::timer::getInterval() );

		count = 0;
		reader->tick();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		REQUIRE( count == 1 );
		REQUIRE( tspkt.timestamp == itime+2*ftl::timer::getInterval() );
		
	}
}
