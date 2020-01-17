#include "catch.hpp"

#define _FTL_NET_UNIVERSE_HPP_

#include <ftl/uuid.hpp>

// Mock the net universe class

namespace ftl {
namespace net {

class Peer {

};

class Universe {
	public:
};

}
}

#include <ftl/streams/netstream.hpp>
#include "../src/netstream.cpp"

using ftl::stream::Net;
using ftl::stream::Stream;
using ftl::config::json_t;

TEST_CASE("ftl::stream::Net post", "[stream]") {
	json_t global = json_t{{"$id","ftl://test"}};
	ftl::config::configure(global);

	json_t cfg = json_t{
		{"$id","ftl://test/1"}
	};

	ftl::net::Universe mocknet;

	Net *s = ftl::create<Net>(cfg, &mocknet);
	s->set("uri", "ftl://dummy");

	REQUIRE(s->begin());

	delete s;
}

