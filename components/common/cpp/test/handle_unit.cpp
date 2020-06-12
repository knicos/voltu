#include "catch.hpp"
#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>
#include <ftl/handle.hpp>

using ftl::Handler;
using ftl::Handle;

TEST_CASE( "Handle release on cancel" ) {
	Handler<int> handler;

	int calls = 0;

	auto h = handler.on([&calls](int i) {
		calls += i;
		return true;
	});

	handler.trigger(5);
	REQUIRE(calls == 5);
	h.cancel();
	handler.trigger(5);
	REQUIRE(calls == 5);
}

TEST_CASE( "Handle release on false return" ) {
	Handler<int> handler;

	int calls = 0;

	auto h = handler.on([&calls](int i) {
		calls += i;
		return false;
	});

	handler.trigger(5);
	REQUIRE(calls == 5);
	handler.trigger(5);
	REQUIRE(calls == 5);
}

TEST_CASE( "Handle multiple triggers" ) {
	Handler<int> handler;

	int calls = 0;

	auto h = handler.on([&calls](int i) {
		calls += i;
		return true;
	});

	handler.trigger(5);
	REQUIRE(calls == 5);
	handler.trigger(5);
	REQUIRE(calls == 10);
}

TEST_CASE( "Handle release on destruct" ) {
	Handler<int> handler;

	int calls = 0;

	{
		auto h = handler.on([&calls](int i) {
			calls += i;
			return true;
		});

		handler.trigger(5);
		REQUIRE(calls == 5);
	}

	handler.trigger(5);
	REQUIRE(calls == 5);
}

TEST_CASE( "Handle moving" ) {
	SECTION("old handle cannot cancel") {
		Handler<int> handler;

		int calls = 0;

		auto h = handler.on([&calls](int i) {
			calls += i;
			return true;
		});

		handler.trigger(5);
		REQUIRE(calls == 5);

		auto h2 = std::move(h);
		h.cancel();

		handler.trigger(5);
		REQUIRE(calls == 10);
	}
}