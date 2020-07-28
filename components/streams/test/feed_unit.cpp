#include <catch.hpp>

#include <nlohmann/json.hpp>
#include <ftl/streams/feed.hpp>

#include <ftl/operators/colours.hpp>

using ftl::config::json_t;

TEST_CASE("ftl::streams::Feed can obtain a frameset", "[]") {
	json_t global = json_t{{"$id","ftl://test"}};
	ftl::config::configure(global);

	json_t cfg1 = json_t{
		{"$id","ftl://test/1"}
	};

	json_t cfg2 = json_t{
		{"$id","ftl://test/2"}
	};

	auto* net = ftl::create<ftl::net::Universe>(cfg1);
	auto* feed = ftl::create<ftl::stream::Feed>(cfg2, net);

	feed->add("./file.ftl");
	feed->add("file:///absolutefile.ftl");
	//feed->add("file://relativefile.ftl");  // This is not allowed
	feed->add("file:/./relativefile.ftl");
	feed->add("file:./relativefile.ftl");
	feed->add("device:dev1");
	feed->add("device:/dev2");
	feed->add("device://dev3");

}
