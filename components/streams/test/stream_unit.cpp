#include "catch.hpp"

#include <ftl/streams/stream.hpp>
#include <nlohmann/json.hpp>

using ftl::stream::Muxer;
using ftl::stream::Broadcast;
using ftl::stream::Stream;
using ftl::config::json_t;

class TestStream : public ftl::stream::Stream {
	public:
	TestStream(nlohmann::json &config) : ftl::stream::Stream(config) {};
	~TestStream() {};

	bool onPacket(const std::function<void(const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &)> &cb) {
		cb_ = cb;
		return true;
	}

	bool post(const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
		available(spkt.streamID) += spkt.channel;
		if (cb_) cb_(spkt, pkt);
		return true;
	}

	bool begin() override { return true; }
	bool end() override { return true; }
	bool active() override { return true; }

	private:
	std::function<void(const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &)> cb_;
};

TEST_CASE("ftl::stream::Muxer()::write", "[stream]") {
	json_t global = json_t{{"$id","ftl://test"}};
	ftl::config::configure(global);

	json_t cfg = json_t{
		{"$id","ftl://test/1"}
	};

	Muxer *mux = ftl::create<Muxer>(cfg);
	REQUIRE(mux);

	SECTION("write with one stream") {
		json_t cfg = json_t{
			{"$id","ftl://test/2"}
		};

		Stream *s = ftl::create<TestStream>(cfg);
		REQUIRE(s);

		mux->add(s);

		ftl::codecs::StreamPacket tspkt = {4,0,0,1,ftl::codecs::Channel::Colour};;

		s->onPacket([&tspkt](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
			tspkt = spkt;
		});

		REQUIRE( !mux->post({4,100,0,1,ftl::codecs::Channel::Colour},{}) );
		REQUIRE( tspkt.timestamp == 0 );
	}

	SECTION("write to previously read") {
		json_t cfg1 = json_t{
			{"$id","ftl://test/2"}
		};
		json_t cfg2 = json_t{
			{"$id","ftl://test/3"}
		};

		Stream *s1 = ftl::create<TestStream>(cfg1);
		REQUIRE(s1);
		Stream *s2 = ftl::create<TestStream>(cfg2);
		REQUIRE(s2);

		mux->add(s1);
		mux->add(s2);

		ftl::codecs::StreamPacket tspkt = {4,0,0,1,ftl::codecs::Channel::Colour};
		mux->onPacket([&tspkt](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
			tspkt = spkt;
		});

		REQUIRE( s1->post({4,100,0,0,ftl::codecs::Channel::Colour},{}) );
		REQUIRE( tspkt.timestamp == 100 );
		REQUIRE( tspkt.frame_number == 0 );

		REQUIRE( s2->post({4,101,0,0,ftl::codecs::Channel::Colour},{}) );
		REQUIRE( tspkt.timestamp == 101 );
		REQUIRE( tspkt.frame_number == 1 );

		ftl::codecs::StreamPacket tspkt2 = {4,0,0,1,ftl::codecs::Channel::Colour};
		ftl::codecs::StreamPacket tspkt3 = {4,0,0,1,ftl::codecs::Channel::Colour};
		s1->onPacket([&tspkt2](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
			tspkt2 = spkt;
		});
		s2->onPacket([&tspkt3](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
			tspkt3 = spkt;
		});

		REQUIRE( mux->post({4,200,1,1,ftl::codecs::Channel::Colour},{}) );
		REQUIRE( tspkt3.timestamp == 200 );
		REQUIRE( tspkt3.frame_number == 0 );
	}
}

TEST_CASE("ftl::stream::Muxer()::read", "[stream]") {
	json_t global = json_t{{"$id","ftl://test"}};
	ftl::config::configure(global);

	json_t cfg = json_t{
		{"$id","ftl://test/1"}
	};

	Muxer *mux = ftl::create<Muxer>(cfg);
	REQUIRE(mux);

	SECTION("read with two writing streams") {
		json_t cfg1 = json_t{
			{"$id","ftl://test/2"}
		};
		json_t cfg2 = json_t{
			{"$id","ftl://test/3"}
		};

		Stream *s1 = ftl::create<TestStream>(cfg1);
		REQUIRE(s1);
		Stream *s2 = ftl::create<TestStream>(cfg2);
		REQUIRE(s2);

		mux->add(s1);
		mux->add(s2);

		ftl::codecs::StreamPacket tspkt = {4,0,0,1,ftl::codecs::Channel::Colour};
		mux->onPacket([&tspkt](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
			tspkt = spkt;
		});

		REQUIRE( s1->post({4,100,0,0,ftl::codecs::Channel::Colour},{}) );
		REQUIRE( tspkt.timestamp == 100 );
		REQUIRE( tspkt.frame_number == 0 );

		REQUIRE( s2->post({4,101,0,0,ftl::codecs::Channel::Colour},{}) );
		REQUIRE( tspkt.timestamp == 101 );
		REQUIRE( tspkt.frame_number == 1 );

		REQUIRE( s1->post({4,102,0,1,ftl::codecs::Channel::Colour},{}) );
		REQUIRE( tspkt.timestamp == 102 );
		REQUIRE( tspkt.frame_number == 2 );

		REQUIRE( s2->post({4,103,0,1,ftl::codecs::Channel::Colour},{}) );
		REQUIRE( tspkt.timestamp == 103 );
		REQUIRE( tspkt.frame_number == 3 );
	}

	SECTION("read consistency with two writing streams") {
		json_t cfg1 = json_t{
			{"$id","ftl://test/2"}
		};
		json_t cfg2 = json_t{
			{"$id","ftl://test/3"}
		};

		Stream *s1 = ftl::create<TestStream>(cfg1);
		REQUIRE(s1);
		Stream *s2 = ftl::create<TestStream>(cfg2);
		REQUIRE(s2);

		mux->add(s1);
		mux->add(s2);

		ftl::codecs::StreamPacket tspkt = {4,0,0,1,ftl::codecs::Channel::Colour};
		mux->onPacket([&tspkt](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
			tspkt = spkt;
		});

		REQUIRE( s1->post({4,100,0,0,ftl::codecs::Channel::Colour},{}) );
		REQUIRE( tspkt.timestamp == 100 );
		REQUIRE( tspkt.frame_number == 0 );

		REQUIRE( s2->post({4,101,0,0,ftl::codecs::Channel::Colour},{}) );
		REQUIRE( tspkt.timestamp == 101 );
		REQUIRE( tspkt.frame_number == 1 );

		REQUIRE( s1->post({4,102,0,0,ftl::codecs::Channel::Colour},{}) );
		REQUIRE( tspkt.timestamp == 102 );
		REQUIRE( tspkt.frame_number == 0 );

		REQUIRE( s2->post({4,103,0,0,ftl::codecs::Channel::Colour},{}) );
		REQUIRE( tspkt.timestamp == 103 );
		REQUIRE( tspkt.frame_number == 1 );
	}
}

TEST_CASE("ftl::stream::Broadcast()::write", "[stream]") {
	json_t global = json_t{{"$id","ftl://test"}};
	ftl::config::configure(global);

	json_t cfg = json_t{
		{"$id","ftl://test/1"}
	};

	Broadcast *mux = ftl::create<Broadcast>(cfg);
	REQUIRE(mux);

	SECTION("write with two streams") {
		json_t cfg1 = json_t{
			{"$id","ftl://test/2"}
		};
		json_t cfg2 = json_t{
			{"$id","ftl://test/3"}
		};

		Stream *s1 = ftl::create<TestStream>(cfg1);
		REQUIRE(s1);
		Stream *s2 = ftl::create<TestStream>(cfg2);
		REQUIRE(s2);

		mux->add(s1);
		mux->add(s2);

		ftl::codecs::StreamPacket tspkt1 = {4,0,0,1,ftl::codecs::Channel::Colour};
		ftl::codecs::StreamPacket tspkt2 = {4,0,0,1,ftl::codecs::Channel::Colour};

		s1->onPacket([&tspkt1](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
			tspkt1 = spkt;
		});
		s2->onPacket([&tspkt2](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
			tspkt2 = spkt;
		});

		REQUIRE( mux->post({4,100,0,1,ftl::codecs::Channel::Colour},{}) );
		REQUIRE( tspkt1.timestamp == 100 );
		REQUIRE( tspkt2.timestamp == 100 );
	}

}
