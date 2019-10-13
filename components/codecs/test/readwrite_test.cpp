#include "catch.hpp"

#include <ftl/codecs/writer.hpp>
#include <ftl/codecs/reader.hpp>
#include <ftl/timer.hpp>

#include <sstream>

using ftl::codecs::Writer;
using ftl::codecs::Reader;
using ftl::codecs::StreamPacket;
using ftl::codecs::Packet;
using ftl::codecs::codec_t;
using ftl::codecs::definition_t;
using ftl::codecs::Channel;

TEST_CASE( "Write and read - Single frame" ) {
	std::stringstream s;
	Writer w(s);

	StreamPacket spkt;
	Packet pkt;

	spkt.channel = Channel::Colour;
	spkt.timestamp = ftl::timer::get_time();
	spkt.streamID = 0;

	pkt.codec = codec_t::JSON;
	pkt.definition = definition_t::Any;
	pkt.block_number = 0;
	pkt.block_total = 1;
	pkt.flags = 0;
	pkt.data = {44,44,44};

	w.begin();
	w.write(spkt, pkt);
	w.end();

	REQUIRE( s.str().size() > 0 );

	int n = 0;

	Reader r(s);
	r.begin();
	bool res = r.read(ftl::timer::get_time()+10, [&n](const StreamPacket &rspkt, const Packet &rpkt) {
		++n;
		REQUIRE(rpkt.codec == codec_t::JSON);
		REQUIRE(rpkt.data.size() == 3);
		REQUIRE(rpkt.data[0] == 44);
		REQUIRE(rspkt.channel == Channel::Colour);
	});
	r.end();

	REQUIRE( n == 1 );
	REQUIRE( !res );
}

TEST_CASE( "Write and read - Multiple frames" ) {
	std::stringstream s;
	Writer w(s);

	StreamPacket spkt;
	Packet pkt;

	spkt.channel = Channel::Colour;
	spkt.timestamp = ftl::timer::get_time();
	spkt.streamID = 0;

	pkt.codec = codec_t::JSON;
	pkt.definition = definition_t::Any;
	pkt.block_number = 0;
	pkt.block_total = 1;
	pkt.flags = 0;
	pkt.data = {44,44,44};

	w.begin();
	w.write(spkt, pkt);
	spkt.timestamp += 50;
	pkt.data = {55,55,55};
	w.write(spkt, pkt);
	spkt.timestamp += 50;
	pkt.data = {66,66,66};
	w.write(spkt, pkt);
	w.end();

	REQUIRE( s.str().size() > 0 );

	int n = 0;

	Reader r(s);
	r.begin();
	bool res = r.read(ftl::timer::get_time()+100, [&n](const StreamPacket &rspkt, const Packet &rpkt) {
		++n;
		REQUIRE(rpkt.codec == codec_t::JSON);
		REQUIRE(rpkt.data.size() == 3);
		REQUIRE(rpkt.data[0] == ((n == 1) ? 44 : (n == 2) ? 55 : 66));
		REQUIRE(rspkt.channel == Channel::Colour);
	});
	r.end();

	REQUIRE( n == 3 );
	REQUIRE( !res );
}

TEST_CASE( "Write and read - Multiple streams" ) {
	std::stringstream s;
	Writer w(s);

	StreamPacket spkt;
	Packet pkt;

	spkt.channel = Channel::Colour;
	spkt.timestamp = ftl::timer::get_time();
	spkt.streamID = 0;

	pkt.codec = codec_t::JSON;
	pkt.definition = definition_t::Any;
	pkt.block_number = 0;
	pkt.block_total = 1;
	pkt.flags = 0;
	pkt.data = {44,44,44};

	w.begin();
	w.write(spkt, pkt);
	spkt.streamID = 1;
	pkt.data = {55,55,55};
	w.write(spkt, pkt);
	w.end();

	REQUIRE( s.str().size() > 0 );

	int n1 = 0;
	int n2 = 0;

	Reader r(s);

	r.onPacket(0, [&n1](const StreamPacket &rspkt, const Packet &rpkt) {
		++n1;
		REQUIRE(rspkt.streamID == 0);
		REQUIRE(rpkt.data.size() == 3);
		REQUIRE(rpkt.data[0] == 44);
	});

	r.onPacket(1, [&n2](const StreamPacket &rspkt, const Packet &rpkt) {
		++n2;
		REQUIRE(rspkt.streamID == 1);
		REQUIRE(rpkt.data.size() == 3);
		REQUIRE(rpkt.data[0] == 55);
	});

	r.begin();
	bool res = r.read(ftl::timer::get_time()+100);
	r.end();

	REQUIRE( n1 == 1 );
	REQUIRE( n2 == 1 );
	REQUIRE( !res );
}

TEST_CASE( "Write and read - Multiple frames with limit" ) {
	std::stringstream s;
	Writer w(s);

	StreamPacket spkt;
	Packet pkt;

	spkt.channel = Channel::Colour;
	spkt.timestamp = ftl::timer::get_time();
	spkt.streamID = 0;

	pkt.codec = codec_t::JSON;
	pkt.definition = definition_t::Any;
	pkt.block_number = 0;
	pkt.block_total = 1;
	pkt.flags = 0;
	pkt.data = {44,44,44};

	w.begin();
	w.write(spkt, pkt);
	spkt.timestamp += 50;
	pkt.data = {55,55,55};
	w.write(spkt, pkt);
	spkt.timestamp += 50;
	pkt.data = {66,66,66};
	w.write(spkt, pkt);
	w.end();

	REQUIRE( s.str().size() > 0 );

	int n = 0;

	Reader r(s);
	r.begin();
	bool res = r.read(ftl::timer::get_time()+50, [&n](const StreamPacket &rspkt, const Packet &rpkt) {
		++n;
		REQUIRE(rpkt.codec == codec_t::JSON);
		REQUIRE(rpkt.data.size() == 3);
		REQUIRE(rpkt.data[0] == ((n == 1) ? 44 : (n == 2) ? 55 : 66));
		REQUIRE(rspkt.channel == Channel::Colour);
	});
	r.end();

	REQUIRE( n == 2 );
	REQUIRE( res );
}

TEST_CASE( "Write and read - Multiple reads" ) {
	std::stringstream s;
	Writer w(s);

	StreamPacket spkt;
	Packet pkt;

	spkt.channel = Channel::Colour;
	spkt.timestamp = ftl::timer::get_time();
	spkt.streamID = 0;

	pkt.codec = codec_t::JSON;
	pkt.definition = definition_t::Any;
	pkt.block_number = 0;
	pkt.block_total = 1;
	pkt.flags = 0;
	pkt.data = {44,44,44};

	w.begin();
	w.write(spkt, pkt);
	spkt.timestamp += 50;
	pkt.data = {55,55,55};
	w.write(spkt, pkt);
	spkt.timestamp += 50;
	pkt.data = {66,66,66};
	w.write(spkt, pkt);
	w.end();

	REQUIRE( s.str().size() > 0 );

	int n = 0;

	Reader r(s);
	r.begin();
	bool res = r.read(ftl::timer::get_time()+50, [&n](const StreamPacket &rspkt, const Packet &rpkt) {
		++n;
		REQUIRE(rpkt.codec == codec_t::JSON);
		REQUIRE(rpkt.data.size() == 3);
		REQUIRE(rpkt.data[0] == ((n == 1) ? 44 : (n == 2) ? 55 : 66));
		REQUIRE(rspkt.channel == Channel::Colour);
	});

	REQUIRE( n == 2 );
	REQUIRE( res );

	n = 0;
	res = r.read(ftl::timer::get_time()+100, [&n](const StreamPacket &rspkt, const Packet &rpkt) {
		++n;
		REQUIRE(rpkt.codec == codec_t::JSON);
		REQUIRE(rpkt.data.size() == 3);
		REQUIRE(rpkt.data[0] == 66 );
		REQUIRE(rspkt.channel == Channel::Colour);
	});
	r.end();

	REQUIRE( n == 1 );
	REQUIRE( !res );
}
