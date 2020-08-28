#include "catch.hpp"

#include <ftl/streams/filestream.hpp>
#include <nlohmann/json.hpp>
#include <ftl/timer.hpp>
#include <ftl/file.hpp>

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
		writer->set("filename", (std::filesystem::temp_directory_path() / "ftl_file_stream_test.ftl").string());
		writer->setMode(File::Mode::Write);

		REQUIRE( writer->begin() );

		REQUIRE( writer->post({4,ftl::timer::get_time(),2,1,ftl::codecs::Channel::Confidence},{ftl::codecs::codec_t::Any, 0, 0, 0, 0, {'f'}}) );

		writer->end();

		reader->set("filename", (std::filesystem::temp_directory_path() / "ftl_file_stream_test.ftl").string());

		ftl::codecs::StreamPacket tspkt = {4,0,0,1,ftl::codecs::Channel::Colour};
		auto h = reader->onPacket([&tspkt](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
			tspkt = spkt;
			return true;
		});
		REQUIRE( reader->begin(false) );

		reader->tick(ftl::timer::get_time()+10);
		reader->end();

		//REQUIRE( tspkt.timestamp == 0 );
		REQUIRE( tspkt.streamID == (uint8_t)2 );
		REQUIRE( tspkt.channel == ftl::codecs::Channel::Confidence );
	}

	SECTION("write read multiple packets at same timestamp") {
		writer->set("filename", (std::filesystem::temp_directory_path() / "ftl_file_stream_test.ftl").string());
		writer->setMode(File::Mode::Write);

		REQUIRE( writer->begin() );

		REQUIRE( writer->post({5,10,0,1,ftl::codecs::Channel::Confidence},{ftl::codecs::codec_t::Any, 0, 0, 0, 0, {'f'}}) );
		REQUIRE( writer->post({5,10,1,1,ftl::codecs::Channel::Depth},{ftl::codecs::codec_t::Any, 0, 0, 0, 0, {'f'}}) );
		REQUIRE( writer->post({5,10,2,1,ftl::codecs::Channel::Screen},{ftl::codecs::codec_t::Any, 0, 0, 0, 0, {'f'}}) );

		writer->end();

		reader->set("filename", (std::filesystem::temp_directory_path() / "ftl_file_stream_test.ftl").string());

		ftl::codecs::StreamPacket tspkt = {5,0,0,1,ftl::codecs::Channel::Colour};
		std::atomic_int count = 0;
		
		auto h = reader->onPacket([&tspkt,&count](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
			tspkt = spkt;
			++count;
			return true;
		});
		REQUIRE( reader->begin(false) );

		reader->tick(ftl::timer::get_time()+10);
		reader->end();

		REQUIRE( count == 3 );
		REQUIRE( tspkt.timestamp > 0 );
		REQUIRE( tspkt.streamID == 2 );
		REQUIRE( tspkt.channel == ftl::codecs::Channel::Screen );
	}

	SECTION("write read multiple packets at different timestamps") {
		writer->set("filename", (std::filesystem::temp_directory_path() / "ftl_file_stream_test.ftl").string());
		writer->setMode(File::Mode::Write);

		REQUIRE( writer->begin() );

		auto time = ftl::timer::get_time();
		REQUIRE( writer->post({4,time,0,1,ftl::codecs::Channel::Confidence},{ftl::codecs::codec_t::Any, 0, 0, 0, 0, {'f'}}) );
		REQUIRE( writer->post({4,time+ftl::timer::getInterval(),0,1,ftl::codecs::Channel::Depth},{ftl::codecs::codec_t::Any, 0, 0, 0, 0, {'f'}}) );
		REQUIRE( writer->post({4,time+2*ftl::timer::getInterval(),0,1,ftl::codecs::Channel::Screen},{ftl::codecs::codec_t::Any, 0, 0, 0, 0, {'f'}}) );

		writer->end();

		reader->set("filename", (std::filesystem::temp_directory_path() / "ftl_file_stream_test.ftl").string());

		ftl::codecs::StreamPacket tspkt = {4,0,0,1,ftl::codecs::Channel::Colour};
		int count = 0;
		auto h = reader->onPacket([&tspkt,&count](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
			tspkt = spkt;
			++count;
			return true;
		});
		REQUIRE( reader->begin(false) );

		reader->tick(ftl::timer::get_time()+ftl::timer::getInterval());
		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		REQUIRE( count == 2 );
		//REQUIRE( tspkt.timestamp == 0 );
		//auto itime = tspkt.timestamp;

		count = 0;
		reader->tick(ftl::timer::get_time()+2*ftl::timer::getInterval());
		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		REQUIRE( count == 2 );
		//REQUIRE( tspkt.timestamp == itime+ftl::timer::getInterval() );

		count = 0;
		reader->tick(ftl::timer::get_time()+3*ftl::timer::getInterval());
		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		REQUIRE( count == 1 );
		//REQUIRE( tspkt.timestamp == itime+2*ftl::timer::getInterval() );
		
	}
}
