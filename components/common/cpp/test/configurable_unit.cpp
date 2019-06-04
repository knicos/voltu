#include "catch.hpp"
#include <ftl/configurable.hpp>

using ftl::Configurable;
using std::string;

SCENARIO( "Configurable::get()" ) {
	GIVEN( "a non-existent property" ) {
		nlohmann::json json = {{"test",5}};
		Configurable cfg(json);

		auto r = cfg.get<int>("test2");
		REQUIRE( !r );
	}

	GIVEN( "a valid property" ) {
		nlohmann::json json = {{"test",5}};
		Configurable cfg(json);

		auto r = cfg.get<int>("test");
		REQUIRE( r );
		REQUIRE( *r == 5 );
	}
}

SCENARIO( "Configurable::on()" ) {
	GIVEN( "a changed property with no handlers" ) {
		nlohmann::json json = {{"test",5}};
		Configurable cfg(json);

		cfg.set("test", 55);
		REQUIRE( *(cfg.get<int>("test")) == 55 );

	}

	GIVEN( "a changed property one handler" ) {
		nlohmann::json json = {{"test",5}};
		Configurable cfg(json);
		bool trig = false;

		cfg.on("test", [&trig](const ftl::config::Event &e) {
			trig = true;
		});

		cfg.set("test", 55);
		REQUIRE( trig );

	}

	GIVEN( "a changed property two handlers" ) {
		nlohmann::json json = {{"test",5}};
		Configurable cfg(json);
		bool trig1 = false;
		bool trig2 = false;

		cfg.on("test", [&trig1](const ftl::config::Event &e) {
			trig1 = true;
		});
		cfg.on("test", [&trig2](const ftl::config::Event &e) {
			trig2 = true;
		});

		cfg.set("test", 55);
		REQUIRE( (trig1 && trig2) );

	}
}
