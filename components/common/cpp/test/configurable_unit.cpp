#include "catch.hpp"
#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>
#include <ftl/configurable.hpp>

#include <nlohmann/json.hpp>

using ftl::Configurable;
using std::string;


SCENARIO( "Configurable::get()" ) {
	GIVEN( "a non-existent property" ) {
		// cppcheck-suppress constStatement
		nlohmann::json json = {{"test",5}};
		Configurable cfg(json);

		auto r = cfg.get<int>("test2");
		REQUIRE( !r );
	}

	GIVEN( "a valid property" ) {
		// cppcheck-suppress constStatement
		nlohmann::json json = {{"test",5}};
		Configurable cfg(json);

		auto r = cfg.get<int>("test");
		REQUIRE( r );
		REQUIRE( *r == 5 );
	}
}

SCENARIO( "Configurable::on()" ) {
	GIVEN( "a changed property with no handlers" ) {
		// cppcheck-suppress constStatement
		nlohmann::json json = {{"test",5}};
		Configurable cfg(json);

		cfg.set("test", 55);
		REQUIRE( *(cfg.get<int>("test")) == 55 );

	}

	GIVEN( "a changed property one handler" ) {
		// cppcheck-suppress constStatement
		nlohmann::json json = {{"test",5}};
		Configurable cfg(json);
		bool trig = false;

		cfg.on("test", [&trig]() {
			trig = true;
		});

		cfg.set("test", 55);
		REQUIRE( trig );

	}

	GIVEN( "a changed property two handlers" ) {
		// cppcheck-suppress constStatement
		nlohmann::json json = {{"test",5}};
		Configurable cfg(json);
		bool trig1 = false;
		bool trig2 = false;

		cfg.on("test", [&trig1]() {
			trig1 = true;
		});
		cfg.on("test", [&trig2]() {
			trig2 = true;
		});

		cfg.set("test", 55);
		REQUIRE( (trig1 && trig2) );

	}
}
