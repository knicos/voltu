#include "catch.hpp"

#include <ftl/data/new_frame.hpp>


using ftl::data::Frame;
using ftl::codecs::Channel;

TEST_CASE("ftl::data::Frame create get", "[Frame]") {
	SECTION("write and read integers") {
		Frame f;
		f.create<int>(Channel::Pose, 55);

		const auto &x = f.get<int>(Channel::Pose);
		REQUIRE( x == 55 );
	}

	SECTION("write and read floats") {
		Frame f;
		f.create<float>(Channel::Pose, 44.0f);

		const auto &x = f.get<float>(Channel::Pose);
		REQUIRE( x == 44.0f );
	}

	SECTION("write and read structures") {
		struct Test {
			int a=44;
			float b=33.0f;
		};
		Frame f;
		f.create<Test>(Channel::Pose, {});

		const auto &x = f.get<Test>(Channel::Pose);
		REQUIRE( x.a == 44 );
		REQUIRE( x.b == 33.0f );
	}

	SECTION("write and read fail") {
		struct Test {
			int a=44;
			float b=33.0f;
		};
		Frame f;
		f.create<Test>(Channel::Pose, {});

		bool err = false;

		try {
			int x = f.get<int>(Channel::Pose);
			REQUIRE(x);
		} catch (...) {
			err = true;
		}
		REQUIRE(err);
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
		const auto &x = f.get<int>(Channel::Pose);
		REQUIRE( x == 55 );

		f.create<int>(Channel::Pose);
		const auto &y = f.get<int>(Channel::Pose);
		REQUIRE( y == 55 );
	}

	SECTION("change of type") {
		Frame f;

		f.create<int>(Channel::Pose, 55);
		auto x = f.getPtr<int>(Channel::Pose);
		REQUIRE( x );
		REQUIRE( *x == 55 );

		f.create<float>(Channel::Pose);
		auto y = f.getPtr<float>(Channel::Pose);
		REQUIRE( y );
		REQUIRE( *y == 0.0f );
	}
}

TEST_CASE("ftl::data::Frame use of parent", "[Frame]") {
	SECTION("get from parent") {
		Frame p;
		Frame f(&p);

		p.create<int>(Channel::Pose, 55);

		auto x = f.getPtr<int>(Channel::Pose);
		REQUIRE( x );
		REQUIRE( *x == 55 );

		auto y = p.getPtr<int>(Channel::Pose);
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

		auto x = f.getPtr<int>(Channel::Pose);
		REQUIRE( x );
		REQUIRE( *x == 66 );

		auto y = p.getPtr<int>(Channel::Pose);
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
		auto x = p.getPtr<int>(Channel::Pose);
		REQUIRE( x );
		REQUIRE( *x == 55 );
		
		f.flush();
		x = p.getPtr<int>(Channel::Pose);
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

// ==== Complex type overload test =============================================

struct TestA {
	int a=55;
};

struct TestB {
	int b=99;
};

struct TestC {
	TestA a;
	TestB b;
};

template <>
TestA &ftl::data::Frame::create<TestA>(ftl::codecs::Channel c) {
	return create<TestC>(c).a;
}

template <>
TestA &ftl::data::Frame::create<TestA>(ftl::codecs::Channel c, const TestA &a) {
	TestC cc;
	cc.a = a;
	return create<TestC>(c, cc).a;
}

template <>
TestB &ftl::data::Frame::create<TestB>(ftl::codecs::Channel c, const TestB &b) {
	TestC cc;
	cc.b = b;
	return create<TestC>(c, cc).b;
}

template <>
const TestA *ftl::data::Frame::getPtr<TestA>(ftl::codecs::Channel c) const noexcept {
	auto *ptr = getPtr<TestC>(c);
	return (ptr) ? &ptr->a : nullptr;
}

template <>
const TestB *ftl::data::Frame::getPtr<TestB>(ftl::codecs::Channel c) const noexcept {
	auto *ptr = getPtr<TestC>(c);
	return (ptr) ? &ptr->b : nullptr;
}

TEST_CASE("ftl::data::Frame Complex Overload", "[Frame]") {
	SECTION("Create and get first type with default") {
		Frame f;
		f.create<TestA>(Channel::Pose);
		
		auto *x = f.getPtr<TestA>(Channel::Pose);
		REQUIRE( x );
		REQUIRE( x->a == 55 );

		auto *y = f.getPtr<TestB>(Channel::Pose);
		REQUIRE( y );
		REQUIRE( y->b == 99 );
	}

	SECTION("Create and get first type with value") {
		Frame f;
		f.create<TestA>(Channel::Pose, {77});
		
		auto *x = f.getPtr<TestA>(Channel::Pose);
		REQUIRE( x );
		REQUIRE( x->a == 77 );

		auto *y = f.getPtr<TestB>(Channel::Pose);
		REQUIRE( y );
		REQUIRE( y->b == 99 );
	}
}
