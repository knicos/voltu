#include "catch.hpp"
#include <ftl/rgbd/channels.hpp>

using ftl::rgbd::Channel;
using ftl::rgbd::Channels;

TEST_CASE("channel casting", "") {
	SECTION("cast channel to channels") {
		Channels cs(Channel::Depth);

        REQUIRE( (unsigned int)cs > 0 );
        REQUIRE( cs.count() == 1 );
	}

    SECTION("cast channels to channel") {
		Channels cs(Channel::Depth);
        Channel c = (Channel)cs;

        REQUIRE( c == Channel::Depth );
	}
}

TEST_CASE("Channel or-ing", "") {
	SECTION("Add channel to channel mask") {
		Channels cs(Channel::Depth);

        cs |= Channel::Right;

        REQUIRE( (cs.count() == 2) );
        REQUIRE( cs.has(Channel::Right) );
        REQUIRE( cs.has(Channel::Depth) );
	}

    SECTION("Combine multiple channels in assignment") {
		Channels cs;

        cs = Channel::Right | Channel::Flow | Channel::Left;

        REQUIRE( (cs.count() == 3) );
        REQUIRE( cs.has(Channel::Right) );
        REQUIRE( cs.has(Channel::Flow) );
        REQUIRE( cs.has(Channel::Left) );
	}

    SECTION("Combine multiple channels at init") {
		Channels cs = Channel::Right | Channel::Flow | Channel::Left;

        REQUIRE( (cs.count() == 3) );
        REQUIRE( cs.has(Channel::Right) );
        REQUIRE( cs.has(Channel::Flow) );
        REQUIRE( cs.has(Channel::Left) );
	}
}

TEST_CASE("Channel adding", "") {
	SECTION("Add channel to channel mask") {
		Channels cs(Channel::Depth);

        cs += Channel::Right;

        REQUIRE( (cs.count() == 2) );
        REQUIRE( cs.has(Channel::Right) );
        REQUIRE( cs.has(Channel::Depth) );
	}

    SECTION("Combine multiple channels in assignment") {
		Channels cs;

        cs = Channel::Right + Channel::Flow + Channel::Left;

        REQUIRE( (cs.count() == 3) );
        REQUIRE( cs.has(Channel::Right) );
        REQUIRE( cs.has(Channel::Flow) );
        REQUIRE( cs.has(Channel::Left) );
	}
}

TEST_CASE("Channel subtracting", "") {
	SECTION("Remove channel from channel mask") {
		Channels cs = Channel::Right | Channel::Flow | Channel::Left;

        cs -= Channel::Flow;

        REQUIRE( (cs.count() == 2) );
        REQUIRE( cs.has(Channel::Right) );
        REQUIRE( cs.has(Channel::Left) );
	}
}
