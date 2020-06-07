#include "catch.hpp"

#include <ftl/data/new_frame.hpp>


using ftl::data::Frame;
using ftl::codecs::Channel;

TEST_CASE("ftl::data::Frame create get", "[Frame]") {
	SECTION("write and read integers") {
		Frame f;
		f.create<int>(Channel::Pose, 55);

		auto x = f.get<int>(Channel::Pose);
		REQUIRE( x );
		REQUIRE( *x == 55 );
	}

	SECTION("write and read floats") {
		Frame f;
		f.create<float>(Channel::Pose, 44.0f);

		auto x = f.get<float>(Channel::Pose);
		REQUIRE( x );
		REQUIRE( *x == 44.0f );
	}

	SECTION("write and read structures") {
		struct Test {
			int a=44;
			float b=33.0f;
		};
		Frame f;
		f.create<Test>(Channel::Pose, {});

		auto x = f.get<Test>(Channel::Pose);
		REQUIRE( x );
		REQUIRE( x->a == 44 );
		REQUIRE( x->b == 33.0f );
	}

	SECTION("write and read fail") {
		struct Test {
			int a=44;
			float b=33.0f;
		};
		Frame f;
		f.create<Test>(Channel::Pose, {});

		auto x = f.get<int>(Channel::Pose);
		REQUIRE( !x );
	}
}

TEST_CASE("ftl::data::Frame isType", "[Frame]") {
	SECTION("is int type") {
		Frame f;
		f.create<int>(Channel::Pose, 55);

		REQUIRE( f.isType<int>(Channel::Pose) );
		REQUIRE( !f.isType<float>(Channel::Pose) );
	}

	SECTION("is struct type") {
		struct Test {
			int a; int b;
		};

		Frame f;
		f.create<Test>(Channel::Pose, {3,4});

		REQUIRE( f.isType<Test>(Channel::Pose) );
		REQUIRE( !f.isType<float>(Channel::Pose) );
	}

	SECTION("missing") {
		Frame f;

		REQUIRE( !f.isType<float>(Channel::Pose) );
	}
}

TEST_CASE("ftl::data::Frame changed", "[Frame]") {
	SECTION("change on create") {
		Frame f;

		REQUIRE( !f.changed(Channel::Pose) );
		f.create<int>(Channel::Pose, 55);
		REQUIRE( f.changed(Channel::Pose) );
	}

	SECTION("no change on untouch") {
		Frame f;

		f.create<int>(Channel::Pose, 55);
		REQUIRE( f.changed(Channel::Pose) );
		f.untouch(Channel::Pose);
		REQUIRE( !f.changed(Channel::Pose) );
	}
}

TEST_CASE("ftl::data::Frame create", "[Frame]") {
	SECTION("same value on create") {
		Frame f;

		f.create<int>(Channel::Pose, 55);
		auto x = f.get<int>(Channel::Pose);
		REQUIRE( x );
		REQUIRE( *x == 55 );

		f.create<int>(Channel::Pose);
		auto y = f.get<int>(Channel::Pose);
		REQUIRE( y );
		REQUIRE( *y == 55 );

		REQUIRE( x == y );
	}

	SECTION("change of type") {
		Frame f;

		f.create<int>(Channel::Pose, 55);
		auto x = f.get<int>(Channel::Pose);
		REQUIRE( x );
		REQUIRE( *x == 55 );

		f.create<float>(Channel::Pose);
		auto y = f.get<float>(Channel::Pose);
		REQUIRE( y );
		REQUIRE( *y == 0.0f );
	}
}

TEST_CASE("ftl::data::Frame use of parent", "[Frame]") {
	SECTION("get from parent") {
		Frame p;
		Frame f(&p);

		p.create<int>(Channel::Pose, 55);

		auto x = f.get<int>(Channel::Pose);
		REQUIRE( x );
		REQUIRE( *x == 55 );

		auto y = p.get<int>(Channel::Pose);
		REQUIRE( x == y );
	}

	SECTION("has from parent") {
		Frame p;
		Frame f(&p);

		p.create<int>(Channel::Pose, 55);
		REQUIRE( f.has(Channel::Pose) );
	}

	SECTION("no change in parent") {
		Frame p;
		Frame f(&p);

		p.create<int>(Channel::Pose, 55);
		p.untouch(Channel::Pose);

		REQUIRE( !f.changed(Channel::Pose) );
		REQUIRE( !p.changed(Channel::Pose) );

		f.set<int>(Channel::Pose, 66);

		REQUIRE( f.changed(Channel::Pose) );
		REQUIRE( !p.changed(Channel::Pose) );

		auto x = f.get<int>(Channel::Pose);
		REQUIRE( x );
		REQUIRE( *x == 66 );

		auto y = p.get<int>(Channel::Pose);
		REQUIRE( y );
		REQUIRE( *y == 55 );
	}
}

TEST_CASE("ftl::data::Frame flush", "[Frame]") {
	SECTION("event on flush") {
		Frame f;

		int event = 0;
		f.on(Channel::Pose, [&event](Frame &frame, Channel c) {
			event++;
			return true;
		});

		f.create<int>(Channel::Pose, 55);
		REQUIRE( event == 0 );

		f.flush();
		REQUIRE( event == 1 );
	}

	SECTION("parent event on flush") {
		Frame p;
		Frame f(&p);

		int event = 0;
		p.on(Channel::Pose, [&event](Frame &frame, Channel c) {
			event++;
			return true;
		});

		f.create<int>(Channel::Pose, 55);
		REQUIRE( event == 0 );

		f.flush();
		REQUIRE( event == 1 );
	}

	SECTION("parent change on flush") {
		Frame p;
		Frame f(&p);

		p.create<int>(Channel::Pose, 55);
		p.flush();

		f.set<int>(Channel::Pose, 66);
		auto x = p.get<int>(Channel::Pose);
		REQUIRE( x );
		REQUIRE( *x == 55 );
		
		f.flush();
		x = p.get<int>(Channel::Pose);
		REQUIRE( x );
		REQUIRE( *x == 66 );
	}

	SECTION("untouched on flush") {
		Frame f;

		f.create<int>(Channel::Pose, 55);
		REQUIRE( f.changed(Channel::Pose) );

		f.flush();
		REQUIRE( !f.changed(Channel::Pose) );
	}
}
