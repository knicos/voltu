/*
 * These tests directly relate to the specification found at:
 *     https://gitlab.utu.fi/nicolas.pope/ftl/-/wikis/Design/Frames
 */

#include "catch.hpp"

#include <ftl/data/new_frame.hpp>

using ftl::data::Session;
using ftl::data::Frame;
using ftl::codecs::Channel;
using ftl::data::ChangeType;
using ftl::data::StorageMode;
using ftl::data::FrameID;

namespace ftl {
namespace data {

class Pool {
	public:
	static Frame make(Session *s, FrameID id, uint64_t ts) { return Frame(nullptr, s, id, ts); }
	static Frame make(Pool *p, Session *s, FrameID id, uint64_t ts) { return Frame(p, s, id, ts); }

	void release(Frame &f);

	Frame allocate(FrameID id, int64_t ts);

	ftl::Handler<ftl::data::Frame&,ftl::codecs::Channel> flush_;
	ftl::Handler<ftl::data::FrameSet&,ftl::codecs::Channel> flush_fs_;
};

}

namespace streams {

// Only Pool can create frames so make a mock Feed.
class Feed {
	public:
	static Frame make(Session *s, FrameID id, uint64_t ts) { return ftl::data::Pool::make(s, id, ts); }
};

}
}

using ftl::streams::Feed;

void ftl::data::Pool::release(Frame &f) {

}

Frame ftl::data::Pool::allocate(FrameID id, int64_t ts) {
	return make(nullptr, id, ts);
}

#define _FTL_DATA_FRAMEPOOL_HPP_
#include <../src/new_frame.cpp>



/* #1.1.1 */
static_assert(sizeof(ftl::codecs::Channel) >= 4, "Channel must be at least 32bit");

/* #1.1.2 */
//static_assert(std::is_integral<decltype(ftl::data::Frame::id)>::value, "Integral ID requried in Frame");
static_assert(std::is_member_function_pointer<decltype(&ftl::data::Frame::id)>::value, "ID is required");
static_assert(std::is_member_function_pointer<decltype(&ftl::data::Frame::timestamp)>::value, "Timestamp is required");

/* #1.1.3  */
static_assert(std::is_member_function_pointer<decltype(&ftl::data::Frame::mutex)>::value, "Frame::mutex is not a member function.");

/* #1.1.4 */
TEST_CASE("ftl::data::Frame encoded data", "[1.1.4]") {
	SECTION("provide encoded data") {
		Frame f = Feed::make(nullptr, FrameID(0,0), 0);
		ftl::codecs::Packet data;
		data.flags = 45;

		f.createChange<int>(Channel::Pose, ftl::data::ChangeType::FOREIGN, data) = 55;
		const auto &x = f.get<int>(Channel::Pose);
		REQUIRE( x == 55 );

		// Data has been moved.
		//REQUIRE(data.size() == 0);
	}

	SECTION("get encoded data") {
		Frame f = Feed::make(nullptr, FrameID(0,0), 0);
		ftl::codecs::Packet data;
		data.flags = 45;

		f.createChange<int>(Channel::Pose, ftl::data::ChangeType::FOREIGN, data);

		auto &data2 = f.getEncoded(Channel::Pose);
		REQUIRE( data2.size() == 1 );
		REQUIRE( data2.front().flags == 45 );
	}
}

/* #1.1.5 */
TEST_CASE("ftl::data::Frame clear encoded on change", "[1.1.5]") {
	SECTION("change by set") {
		Frame f = Feed::make(nullptr, FrameID(0,0), 0);
		ftl::codecs::Packet data;
		data.flags = 45;

		f.createChange<int>(Channel::Pose, ftl::data::ChangeType::FOREIGN, data);
		f.store();

		auto &data2 = f.getEncoded(Channel::Pose);
		REQUIRE( data2.size() == 1 );
		
		f.set<int>(Channel::Pose) = 66;
		REQUIRE(f.getEncoded(Channel::Pose).size() == 0);
	}

	SECTION("change by create") {
		Frame f = Feed::make(nullptr, FrameID(0,0), 0);
		ftl::codecs::Packet data;
		data.flags = 45;

		f.createChange<int>(Channel::Pose, ftl::data::ChangeType::FOREIGN, data);
		f.store();

		auto &data2 = f.getEncoded(Channel::Pose);
		REQUIRE( data2.size() == 1 );
		
		f.create<int>(Channel::Pose) = 66;
		REQUIRE(f.getEncoded(Channel::Pose).size() == 0);
	}
}

struct Test {
	int a=44;
	float b=33.0f;

	MSGPACK_DEFINE(a,b);
};

/* #1.2.1 */
TEST_CASE("ftl::data::Frame create get", "[Frame]") {
	SECTION("write and read integers") {
		Frame f = Feed::make(nullptr, FrameID(0,0), 0);
		f.store();

		f.create<int>(Channel::Pose) = 55;

		const auto &x = f.get<int>(Channel::Pose);
		REQUIRE( x == 55 );
	}

	SECTION("write and read floats") {
		Frame f = Feed::make(nullptr, FrameID(0,0), 0);
		f.store();

		f.create<float>(Channel::Pose) = 44.0f;

		const auto &x = f.get<float>(Channel::Pose);
		REQUIRE( x == 44.0f );
	}

	SECTION("write and read structures") {
		Frame f = Feed::make(nullptr, FrameID(0,0), 0);
		f.store();

		f.create<Test>(Channel::Pose) = {};

		const auto &x = f.get<Test>(Channel::Pose);
		REQUIRE( x.a == 44 );
		REQUIRE( x.b == 33.0f );
	}

	SECTION("is int type") {
		Frame f = Feed::make(nullptr, FrameID(0,0), 0);
		f.store();

		f.create<int>(Channel::Pose) = 55;

		REQUIRE( f.isType<int>(Channel::Pose) );
		REQUIRE( !f.isType<float>(Channel::Pose) );
	}

	SECTION("is struct type") {
		Frame f = Feed::make(nullptr, FrameID(0,0), 0);
		f.store();

		f.create<Test>(Channel::Pose) = {3,4};

		REQUIRE( f.isType<Test>(Channel::Pose) );
		REQUIRE( !f.isType<float>(Channel::Pose) );
	}

	SECTION("missing") {
		Frame f = Feed::make(nullptr, FrameID(0,0), 0);

		REQUIRE( !f.isType<float>(Channel::Pose) );
	}
}

/* #1.2.2 */
TEST_CASE("ftl::data::registerChannel", "[Frame]") {
	SECTION("register typed channel and valid create") {
		Frame f = Feed::make(nullptr, FrameID(0,0), 0);
		f.store();

		ftl::data::make_channel<float>(Channel::Colour, "colour", ftl::data::StorageMode::PERSISTENT);
		f.create<float>(Channel::Colour) = 5.0f;
		REQUIRE( f.get<float>(Channel::Colour) == 5.0f );

		ftl::data::clearRegistry();
	}

	SECTION("register typed channel and invalid create") {
		Frame f = Feed::make(nullptr, FrameID(0,0), 0);
		f.store();

		ftl::data::make_channel<float>(Channel::Colour, "colour", ftl::data::StorageMode::PERSISTENT);

		bool err = false;
		try {
			f.create<int>(Channel::Colour) = 5;
		} catch(const ftl::exception &e) {
			e.ignore();
			err = true;
		}
		REQUIRE( err );

		ftl::data::clearRegistry();
	}

	SECTION("register void for any type") {
		Frame f = Feed::make(nullptr, FrameID(0,0), 0);
		f.store();

		ftl::data::make_channel<void>(Channel::Colour, "colour", ftl::data::StorageMode::PERSISTENT);
	
		f.create<int>(Channel::Colour) = 5;
		REQUIRE( f.get<int>(Channel::Colour) == 5 );

		ftl::data::clearRegistry();
	}
}

/* #1.2.3 */
TEST_CASE("ftl::data::Frame type failure") {
	SECTION("write and read fail") {
		Frame f = Feed::make(nullptr, FrameID(0,0), 0);
		f.store();
		f.create<Test>(Channel::Pose) = {};

		bool err = false;

		try {
			f.get<int>(Channel::Pose);
		} catch (const ftl::exception &e) {
			e.ignore();
			err = true;
		}
		REQUIRE(err);
	}

	SECTION("same value on create") {
		Frame f = Feed::make(nullptr, FrameID(0,0), 0);
		f.store();

		f.create<int>(Channel::Pose) = 55;
		const auto &x = f.get<int>(Channel::Pose);
		REQUIRE( x == 55 );

		f.create<int>(Channel::Pose);
		const auto &y = f.get<int>(Channel::Pose);
		REQUIRE( y == 55 );
	}

	SECTION("change of type by recreate") {
		Frame f = Feed::make(nullptr, FrameID(0,0), 0);
		f.store();

		f.create<int>(Channel::Pose) = 55;
		auto x = f.getPtr<int>(Channel::Pose);
		REQUIRE( x );
		REQUIRE( *x == 55 );

		f.create<float>(Channel::Pose);
		auto y = f.getPtr<float>(Channel::Pose);
		REQUIRE( y );
		REQUIRE( *y == 0.0f );
	}
}

/* #1.2.4 UNTESTED */

/* #1.2.5 */
TEST_CASE("ftl::data::Frame persistent data", "[1.2.5]") {
	ftl::data::make_channel<int>(Channel::Density, "density", ftl::data::StorageMode::PERSISTENT);

	SECTION("persistent through createChange") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);

		f.createChange<int>(Channel::Density, ChangeType::FOREIGN) = 44;
		f.store();	
		REQUIRE( p.get<int>(Channel::Density) == 44 );
	}

	// These are not valid as per #3.2.5
	/*SECTION("persistent via create") {
		Session p;
		Frame f(&p);

		f.create<int>(Channel::Density, 44);
		f.store();	
		REQUIRE( p.get<int>(Channel::Density) == 44 );
	}

	SECTION("persistent via set") {
		Session p;
		Frame f(&p);

		f.create<int>(Channel::Density, 44);
		f.set<int>(Channel::Density, 45);
		f.store();	
		REQUIRE( p.get<int>(Channel::Density) == 45 );
	}*/

	SECTION("available in other frame") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);

		f.createChange<int>(Channel::Density, ChangeType::FOREIGN) = 44;	
		f.store();	

		Frame f2 = Feed::make(&p, FrameID(0,0), 0);
		REQUIRE( f2.get<int>(Channel::Density) == 44 );
	}

	SECTION("get from parent") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);
		f.store();

		p.create<int>(Channel::Pose) = 55;

		auto x = f.getPtr<int>(Channel::Pose);
		REQUIRE( x );
		REQUIRE( *x == 55 );

		auto y = p.getPtr<int>(Channel::Pose);
		REQUIRE( x == y );
	}

	SECTION("get from parent not ptr") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);
		f.store();

		p.create<int>(Channel::Pose) = 55;

		auto x = f.get<int>(Channel::Pose);
		REQUIRE( x == 55 );
	}

	SECTION("has from parent") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);
		f.store();

		p.create<int>(Channel::Pose) = 55;
		REQUIRE( f.has(Channel::Pose) );
	}

	SECTION("no change in parent") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);
		f.store();

		p.create<int>(Channel::Pose) = 55;
		p.untouch(Channel::Pose);

		REQUIRE( !f.changed(Channel::Pose) );
		REQUIRE( !p.changed(Channel::Pose) );

		f.set<int>(Channel::Pose) = 66;

		REQUIRE( f.changed(Channel::Pose) );
		REQUIRE( !p.changed(Channel::Pose) );

		auto x = f.getPtr<int>(Channel::Pose);
		REQUIRE( x );
		REQUIRE( *x == 66 );

		auto y = p.getPtr<int>(Channel::Pose);
		REQUIRE( y );
		REQUIRE( *y == 55 );
	}

	ftl::data::clearRegistry();
}

/* #1.2.6 */
TEST_CASE("ftl::data::Frame transient data", "[1.2.6]") {
	ftl::data::make_channel<int>(Channel::Density, "density", ftl::data::StorageMode::TRANSIENT);

	SECTION("not persistent after store") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);

		f.createChange<int>(Channel::Density, ChangeType::FOREIGN) = 44;
		f.store();	
		
		REQUIRE( !p.has(Channel::Density) );
	}

	ftl::data::clearRegistry();
}

/* #1.2.7 */
TEST_CASE("ftl::data::Frame aggregate data", "[1.2.7]") {
	ftl::data::make_channel<void>(Channel::Density, "density", ftl::data::StorageMode::AGGREGATE);

	SECTION("not persistent after store") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);

		f.createChange<std::list<int>>(Channel::Density, ChangeType::FOREIGN) = {44};
		f.store();	
		
		REQUIRE( !p.has(Channel::Density) );
	}

	// TODO: Check elsewhere that the changes are since last frame, not
	// applicable as part of this unit test.

	SECTION("aggregate channels actually aggregate with createChange") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);

		f.createChange<std::list<int>>(Channel::Density, ChangeType::FOREIGN) = {34};
		f.createChange<std::list<int>>(Channel::Density, ChangeType::FOREIGN) = {55};
		f.createChange<std::list<int>>(Channel::Density, ChangeType::FOREIGN) = {12,89};
		f.store();

		auto list = f.get<std::list<int>>(Channel::Density).begin();
		REQUIRE( *(list++) == 34 );
		REQUIRE( *(list++) == 55 );
		REQUIRE( *(list++) == 12 );
		REQUIRE( *(list++) == 89 );
	}

	SECTION("non aggregate channels do not aggregate with createChange") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);

		f.createChange<std::list<int>>(Channel::Colour, ChangeType::FOREIGN) = {34};
		f.createChange<std::list<int>>(Channel::Colour, ChangeType::FOREIGN) = {55};
		f.createChange<std::list<int>>(Channel::Colour, ChangeType::FOREIGN) = {12,89};
		f.store();

		auto list = f.get<std::list<int>>(Channel::Colour).begin();
		REQUIRE( *(list++) == 12 );
		REQUIRE( *(list++) == 89 );
	}

	SECTION("aggregate channels allow move aggregate with createChange") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);

		std::list<int> data1 = {34};
		std::list<int> data2 = {55};

		f.createChange<std::list<int>>(Channel::Density, ChangeType::FOREIGN) = std::move(data1);
		f.createChange<std::list<int>>(Channel::Density, ChangeType::FOREIGN) = std::move(data2);
		f.store();

		auto list = f.get<std::list<int>>(Channel::Density).begin();
		REQUIRE( *(list++) == 34 );
		REQUIRE( *(list++) == 55 );
		REQUIRE( data1.size() == 0 );
		REQUIRE( data2.size() == 0 );
	}

	SECTION("aggregate channels actually aggregate with create") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);
		f.store();

		f.create<std::list<int>>(Channel::Density) = {34};
		f.create<std::list<int>>(Channel::Density) = {55};
		f.create<std::list<int>>(Channel::Density) = {12,89};

		auto list = f.get<std::list<int>>(Channel::Density).begin();
		REQUIRE( *(list++) == 34 );
		REQUIRE( *(list++) == 55 );
		REQUIRE( *(list++) == 12 );
		REQUIRE( *(list++) == 89 );
	}

	SECTION("non aggregate channels do not aggregate with create") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);
		f.store();

		f.create<std::list<int>>(Channel::Colour) = {34};
		f.create<std::list<int>>(Channel::Colour) = {55};
		f.create<std::list<int>>(Channel::Colour) = {12,89};

		auto list = f.get<std::list<int>>(Channel::Colour).begin();
		REQUIRE( *(list++) == 12 );
		REQUIRE( *(list++) == 89 );
	}

	SECTION("aggregate channels actually aggregate with set") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);
		f.store();

		f.create<std::list<int>>(Channel::Density) = {34};
		f.set<std::list<int>>(Channel::Density) = {55};
		f.set<std::list<int>>(Channel::Density) = {12,89};

		auto list = f.get<std::list<int>>(Channel::Density).begin();
		REQUIRE( *(list++) == 34 );
		REQUIRE( *(list++) == 55 );
		REQUIRE( *(list++) == 12 );
		REQUIRE( *(list++) == 89 );
	}

	SECTION("non aggregate channels do not aggregate with set") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);
		f.store();

		f.create<std::list<int>>(Channel::Colour) = {34};
		f.set<std::list<int>>(Channel::Colour) = {55};
		f.set<std::list<int>>(Channel::Colour) = {12,89};

		auto list = f.get<std::list<int>>(Channel::Colour).begin();
		REQUIRE( *(list++) == 12 );
		REQUIRE( *(list++) == 89 );
	}

	ftl::data::clearRegistry();
}

/* #1.2.8 Not applicable as a unit test of Frame. */

/* #1.2.9 */
TEST_CASE("ftl::data::Frame aggregate lists", "[1.2.9]") {
	ftl::data::make_channel<void>(Channel::Density, "density", ftl::data::StorageMode::AGGREGATE);

	SECTION("only allow stl list container") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);
		f.store();

		f.create<std::list<int>>(Channel::Density) = {44};

		bool err = false;

		try {
			f.create<int>(Channel::Density);
		} catch (const ftl::exception &e) {
			e.ignore();
			err = true;
		}

		REQUIRE( err );
	}

	ftl::data::clearRegistry();
}

/* #1.3 Not applicable as a unit test of Frame. */

/* #2.1.1 */
static_assert(!std::is_default_constructible<Frame>::value, "Must not have default construction");
// TODO: Check for privacy of actual constructor? Proposed for future feature of C++ but not yet.

/* #2.1.2 */
static_assert(!std::is_copy_constructible<Frame>::value, "Must not have a copy constructor");
static_assert(!std::is_copy_assignable<Frame>::value, "Must not allow copy assignment");

/* #2.1.3 */
static_assert(std::is_move_constructible<Frame>::value, "Must have a move constructor");
static_assert(std::is_move_assignable<Frame>::value, "Must allow move assignment");

/* #2.1.4 Not applicable as a unit test of Frame. */

/* #2.1.5 Not applicable as a unit test of Frame. */

/* #2.1.6 Not applicable as a unit test of Frame. */

/* #2.1.7 Not applicable as a unit test of Frame. */

/* #2.1.8 */
TEST_CASE("ftl::data::Frame merging", "[2.1.8]") {
	SECTION("merge replaces data in destination") {
		Frame f1 = Feed::make(nullptr, FrameID(0,0), 0);
		Frame f2 = Feed::make(nullptr, FrameID(0,0), 0);
		f1.store();
		f2.store();

		f1.create<int>(Channel::Colour) = 43;
		f1.create<int>(Channel::Colour2) = 77;

		f2.create<int>(Channel::Colour2) = 88;

		f2.merge(f1);

		REQUIRE( f2.get<int>(Channel::Colour2) == 77 );
	}

	SECTION("new items are created") {
		Frame f1 = Feed::make(nullptr, FrameID(0,0), 0);
		Frame f2 = Feed::make(nullptr, FrameID(0,0), 0);
		f1.store();
		f2.store();

		f1.create<int>(Channel::Colour) = 43;
		f1.create<int>(Channel::Colour2) = 77;

		f2.create<int>(Channel::Colour2) = 88;

		f2.merge(f1);

		REQUIRE( f2.get<int>(Channel::Colour) == 43 );
	}

	SECTION("old items remain") {
		Frame f1 = Feed::make(nullptr, FrameID(0,0), 0);
		Frame f2 = Feed::make(nullptr, FrameID(0,0), 0);
		f1.store();
		f2.store();

		f1.create<int>(Channel::Colour2) = 77;

		f2.create<int>(Channel::Colour) = 43;
		f2.create<int>(Channel::Colour2) = 88;

		f2.merge(f1);

		REQUIRE( f2.get<int>(Channel::Colour) == 43 );
	}

	SECTION("flushed status is removed") {
		Frame f1 = Feed::make(nullptr, FrameID(0,0), 0);
		Frame f2 = Feed::make(nullptr, FrameID(0,0), 0);
		f1.store();
		f2.store();

		f1.create<int>(Channel::Colour) = 43;
		f1.flush();

		REQUIRE( f1.flushed(Channel::Colour) );

		f2.merge(f1);

		REQUIRE( !f2.flushed(Channel::Colour) );
		REQUIRE( f2.has(Channel::Colour) );
	}
}

/* #2.1.9 */
TEST_CASE("ftl::data::Frame merge is change", "[2.1.9]") {
	SECTION("merges are marked as changes") {
		Frame f1 = Feed::make(nullptr, FrameID(0,0), 0);
		Frame f2 = Feed::make(nullptr, FrameID(0,0), 0);
		f1.store();
		f2.store();

		f1.create<int>(Channel::Colour) = 43;
		f2.create<int>(Channel::Colour2) = 88;
		f2.untouch(Channel::Colour2);
		f2.merge(f1);

		REQUIRE( f2.getChangeType(Channel::Colour) == ChangeType::PRIMARY );
		REQUIRE( !f2.changed(Channel::Colour2) );
	}
}

/* #2.1.10 Unimplemented, merge is move only. This tests for the move instead */
TEST_CASE("ftl::data::Frame merge moves encoded", "[2.1.10]") {
	SECTION("encoded data moved") {
		Frame f1 = Feed::make(nullptr, FrameID(0,0), 0);
		Frame f2 = Feed::make(nullptr, FrameID(0,0), 0);

		ftl::codecs::Packet data;
		data.flags = 45;
		f1.createChange<int>(Channel::Colour, ChangeType::FOREIGN, data);
		f2.merge(f1);

		REQUIRE( f2.getEncoded(Channel::Colour).size() == 1 );
		REQUIRE( !f1.has(Channel::Colour) );
	}
}

/* #2.2.1 */
TEST_CASE("ftl::data::Frame modify after flush", "[2.2.1]") {
	SECTION("create fails after flush") {
		Frame f = Feed::make(nullptr, FrameID(0,0), 0);
		f.store();

		f.create<int>(Channel::Colour) = 89;
		f.flush();

		bool err = false;
		try {
			f.create<int>(Channel::Colour) = 90;
		} catch (const ftl::exception &e) {
			e.ignore();
			err = true;
		}

		REQUIRE( err );
	}

	SECTION("set fails after flush") {
		Frame f = Feed::make(nullptr, FrameID(0,0), 0);
		f.store();

		f.create<int>(Channel::Colour) = 89;
		f.flush();

		bool err = false;
		try {
			f.set<int>(Channel::Colour) = 90;
		} catch (const ftl::exception &e) {
			e.ignore();
			err = true;
		}

		REQUIRE( err );
	}

	SECTION("createChange fails after flush") {
		Frame f = Feed::make(nullptr, FrameID(0,0), 0);
		f.store();

		f.create<int>(Channel::Colour) = 89;
		f.flush();

		bool err = false;
		try {
			f.createChange<int>(Channel::Colour, ChangeType::FOREIGN) = 90;
		} catch (const ftl::exception &e) {
			e.ignore();
			err = true;
		}

		REQUIRE( err );
	}

	SECTION("channel marked readonly after flush") {
		Frame f = Feed::make(nullptr, FrameID(0,0), 0);
		f.store();

		f.create<int>(Channel::Colour) = 89;
		f.flush();
		REQUIRE( f.readonly(Channel::Colour) );
	}
}

/* #2.2.2 FIXME: Specification needs review. */

/* #2.2.3 */
TEST_CASE("ftl::data::Frame multiple flush", "[Frame]") {
	SECTION("fail on multiple frame flush") {
		Frame f = Feed::make(nullptr, FrameID(0,0), 0);
		f.store();

		f.create<int>(Channel::Colour) = 89;
		f.flush();

		bool err = false;
		try {
			f.flush();
		} catch (const ftl::exception &e) {
			e.ignore();
			err = true;
		}

		REQUIRE( err );
	}
}

/* #2.2.4 */
TEST_CASE("ftl::data::Frame locality of changes", "[2.2.4]") {
	ftl::data::make_channel<int>(Channel::Density, "density", ftl::data::StorageMode::PERSISTENT);

	SECTION("persistent after flush only for primary frame") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);
		f.store();

		f.create<int>(Channel::Density) = 44;
		f.flush();

		bool err=false;

		try {		
			p.get<int>(Channel::Density);
		} catch(const ftl::exception &e) {
			e.ignore();
			err = true;
		}
		REQUIRE( !err );
	}

	// FIXME: Need a way to change frame mode or generate response frame.
	/*SECTION("not persistent after flush only for response frame") {
		Session p;
		Frame ff = Feed::make(&p, FrameID(0,0), 0);
		ff.store();
		Frame f = ff.response();

		f.create<int>(Channel::Density) = 44;
		f.flush();

		bool err=false;

		try {		
			p.get<int>(Channel::Density);
		} catch(const ftl::exception &e) {
			e.ignore();
			err = true;
		}
		REQUIRE( err );
	}*/

	SECTION("not persistent without store") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);
		f.store();

		f.create<int>(Channel::Density) = 44;

		bool err=false;

		try {		
			p.get<int>(Channel::Density);
		} catch(const ftl::exception &e) {
			e.ignore();
			err = true;
		}
		REQUIRE( err );
	}

	ftl::data::clearRegistry();
}

/* #2.2.5 */
TEST_CASE("ftl::data::Frame changed status", "[2.2.5]") {
	SECTION("change on create") {
		Frame f = Feed::make(nullptr, FrameID(0,0), 0);
		f.store();

		REQUIRE( !f.changed(Channel::Pose) );
		f.create<int>(Channel::Pose) = 55;
		REQUIRE( f.changed(Channel::Pose) );
	}

	SECTION("no change on untouch") {
		Frame f = Feed::make(nullptr, FrameID(0,0), 0);
		f.store();

		f.create<int>(Channel::Pose) = 55;
		REQUIRE( f.changed(Channel::Pose) );
		f.untouch(Channel::Pose);
		REQUIRE( !f.changed(Channel::Pose) );
	}
}

/* #2.3.1 Not applicable as a unit test of Frame. */

/* #2.3.2 Not applicable as a unit test of Frame. */

/* #2.3.3 */
TEST_CASE("ftl::data::Frame change type", "[2.3.3]") {
	SECTION("changes are local type") {
		Frame f = Feed::make(nullptr, FrameID(0,0), 0);
		f.store();

		REQUIRE( !f.changed(Channel::Pose) );
		f.create<int>(Channel::Pose) = 55;
		REQUIRE( f.getChangeType(Channel::Pose) == ChangeType::PRIMARY );
	}

	SECTION("local change overrides foreign change") {
		Frame f = Feed::make(nullptr, FrameID(0,0), 0);

		f.createChange<int>(Channel::Pose, ChangeType::FOREIGN) = 55;
		REQUIRE( f.getChangeType(Channel::Pose) == ChangeType::FOREIGN );
		f.store();

		f.set<int>(Channel::Pose) = 66;
		REQUIRE( f.getChangeType(Channel::Pose) == ChangeType::PRIMARY );
	}

	SECTION("local change overrides completed change") {
		Frame f = Feed::make(nullptr, FrameID(0,0), 0);

		f.createChange<int>(Channel::Pose, ChangeType::COMPLETED) = 55;
		REQUIRE( f.getChangeType(Channel::Pose) == ChangeType::COMPLETED );
		f.store();
		f.set<int>(Channel::Pose) = 66;
		REQUIRE( f.getChangeType(Channel::Pose) == ChangeType::PRIMARY );
	}
}

/* #2.3.4 Not applicable as a unit test of Frame. */

/* #2.3.5 Not applicable as a unit test of Frame. */

/* #2.3.6 Not applicable as a unit test of Frame. */

/* #2.3.7 Not applicable as a unit test of Frame. */

/* #3.1.1 Not applicable as a unit test of Frame. */

/* #3.1.2 Not applicable as a unit test of Frame. */

/* #3.1.3 Not applicable as a unit test of Frame. */

/* #3.1.4 */
TEST_CASE("ftl::data::Frame override of persistent", "[3.1.4]") {
	SECTION("local changes override persistent data") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);
		f.store();

		p.create<int>(Channel::Colour) = 44;

		// Note: Get without local create
		REQUIRE( f.get<int>(Channel::Colour) == 44 );

		// Note: set without create when exists in session store
		f.set<int>(Channel::Colour) = 66;
		REQUIRE( f.get<int>(Channel::Colour) == 66 );
	}
}

/* #3.1.5 Not applicable as a unit test of Frame. */

/* #3.1.6 FIXME: Specification needs review */

/* #3.1.7 Implicit in other tests. */

/* #3.1.8 Not applicable as a unit test of Frame. */

/* #3.2.1 Not applicable as a unit test of Frame. */

/* #3.2.2 Not applicable as a unit test of Frame. */

/* #3.2.3 Not applicable as a unit test of Frame. */

/* #3.2.4 Not applicable as a unit test of Frame. */

/* #3.2.5 */
TEST_CASE("ftl::data::Frame initial store", "[3.2.5]") {
	SECTION("cannot create before store") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);

		bool err = false;
		try {
			f.create<int>(Channel::Colour) = 55;
		} catch (const ftl::exception &e) {
			e.ignore();
			err = true;
		}
		REQUIRE( err );
	}

	SECTION("can createChange before store") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);

		f.createChange<int>(Channel::Colour, ChangeType::FOREIGN) = 89;
	}

	SECTION("cannot createChange after store") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);

		f.store();

		bool err = false;
		try {
			f.createChange<int>(Channel::Colour, ChangeType::FOREIGN);
		} catch (const ftl::exception &e) {
			e.ignore();
			err = true;
		}
		REQUIRE( err );
	}

	SECTION("cannot store twice") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);

		f.store();

		bool err = false;
		try {
			f.store();
		} catch (const ftl::exception &e) {
			e.ignore();
			err = true;
		}
		REQUIRE( err );
	}

	SECTION("cannot flush before store") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);

		bool err = false;
		try {
			f.flush();
		} catch (const ftl::exception &e) {
			e.ignore();
			err = true;
		}
		REQUIRE( err );
	}
}

/* #3.3.1 Not applicable as a unit test of Frame. */

/* #3.3.2 Not applicable as a unit test of Frame. */

/* #3.3.3 See #3.2.5 */

/* #3.3.2 Not applicable as a unit test of Frame. However, see #3.2.5 */

/* #3.4.1 */
TEST_CASE("ftl::data::Frame change events", "[3.4.1]") {
	SECTION("event on store of foreign change") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);

		int event = 0;
		auto h = f.onChange([&event](Frame &frame, Channel c) {
			event++;
			return true;
		});

		f.createChange<int>(Channel::Pose, ChangeType::FOREIGN);
		REQUIRE( event == 0 );

		f.store();
		REQUIRE( event == 1 );
	}

	SECTION("event on store of completed change") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);

		int event = 0;
		auto h = f.onChange([&event](Frame &frame, Channel c) {
			event++;
			return true;
		});

		f.createChange<int>(Channel::Pose, ChangeType::COMPLETED);
		REQUIRE( event == 0 );

		f.store();
		REQUIRE( event == 1 );
	}

	SECTION("event on store of foreign change with flush") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);

		int event = 0;
		auto h = f.onChange([&event](Frame &frame, Channel c) {
			event++;
			return true;
		});

		f.createChange<int>(Channel::Pose, ChangeType::FOREIGN);
		REQUIRE( event == 0 );

		f.store();
		f.flush();
		REQUIRE( event == 1 );
	}

	SECTION("No event on flush of response frame") {
		ftl::data::Pool p;
		Session s;
		Frame f = ftl::data::Pool::make(&p, &s, FrameID(0,0), 0);

		int event = 0;
		auto h = f.onChange([&event](Frame &frame, Channel c) {
			event++;
			return true;
		});

		{
			auto response = f.response();
			REQUIRE( event == 0 );
			response.create<int>(Channel::Control) = 55;
		}
		REQUIRE( event == 0 );
	}
}

/* #3.4.2 Not applicable as a unit test of Frame. See #3.2.5 */

/* #3.4.3 Not applicable as a unit test of Frame. See #3.2.5 */

/* #3.4.4 */
TEST_CASE("ftl::data::Frame parallel change events", "[3.4.4]") {
	SECTION("event for each of multiple changes") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);

		int event = 0;
		auto h = f.onChange([&event](Frame &frame, Channel c) {
			event++;
			return true;
		});

		f.createChange<int>(Channel::Pose, ChangeType::FOREIGN);
		f.createChange<int>(Channel::Colour, ChangeType::FOREIGN);
		f.createChange<int>(Channel::Depth, ChangeType::FOREIGN);
		REQUIRE( event == 0 );

		f.store();
		REQUIRE( event == 3 );
	}
}

/* #3.4.5 see above test, #3.4.4 */

/* #3.4.6 */
TEST_CASE("ftl::data::Frame aggregate changes", "[3.4.6]") {
	ftl::data::make_channel<std::list<int>>(Channel::Density, "density", ftl::data::StorageMode::AGGREGATE);

	SECTION("multiple changes cause single event") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);

		int event = 0;
		auto h = f.onChange([&event](Frame &frame, Channel c) {
			event++;
			return true;
		});

		f.createChange<std::list<int>>(Channel::Density, ChangeType::FOREIGN) = {34};
		f.createChange<std::list<int>>(Channel::Density, ChangeType::FOREIGN) = {55};
		f.createChange<std::list<int>>(Channel::Density, ChangeType::FOREIGN) = {12,89};
		REQUIRE( event == 0 );

		f.store();
		REQUIRE( event == 1 );
	}

	ftl::data::clearRegistry();
}

/* #3.4.7 */
//static_assert(std::is_same<decltype(Frame::onChange),ftl::Handle(ftl::codecs::Channel, const std::function<bool(Frame&,ftl::codecs::Channel)> &)>::value, "Wrong event handler type");

/* #3.4.8 Not applicable as a unit test of Frame. */

/* #4.1.1 */
TEST_CASE("ftl::data::Frame flush events", "[4.1.1]") {
	SECTION("event on flush") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);
		f.store();

		int event = 0;
		auto h = f.onFlush([&event](Frame &frame, Channel c) {
			event++;
			return true;
		});

		f.create<int>(Channel::Pose) = 55;
		REQUIRE( event == 0 );

		f.flush();
		REQUIRE( event == 1 );
	}

	SECTION("parent event on flush") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);
		f.store();

		int event = 0;
		auto h = p.onFlush([&event](Frame &frame, Channel c) {
			event++;
			return true;
		});

		f.create<int>(Channel::Pose) = 55;
		REQUIRE( event == 0 );

		f.flush();
		REQUIRE( event == 1 );
	}
}

/* #4.1.2 */
TEST_CASE("ftl::data::Frame flush per channel", "[4.1.2]") {
	SECTION("event on flush of channel") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);
		f.store();

		int event = 0;
		auto h = f.onFlush([&event](Frame &frame, Channel c) {
			event++;
			return true;
		});

		f.create<int>(Channel::Pose) = 55;
		f.create<int>(Channel::Colour) = 45;
		REQUIRE( event == 0 );

		f.flush(Channel::Pose);
		REQUIRE( event == 1 );
	}

	SECTION("flushed channel readonly") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);
		f.store();

		f.create<int>(Channel::Pose) = 55;
		f.create<int>(Channel::Colour) = 45;

		f.flush(Channel::Pose);
		REQUIRE( f.readonly(Channel::Pose) );
		REQUIRE( !f.readonly(Channel::Colour) );
	}
}

/* #4.1.3 Not applicable as a unit test of Frame. */

/* #4.1.4 Not applicable as a unit test of Frame. */

/* #4.1.5 Not applicable as a unit test of Frame. */

/* #4.1.6 */
TEST_CASE("ftl::data::Frame flush on destruct", "[4.1.6]") {
	SECTION("flush a non-flushed frame on destruct") {
		Session p;

		int event = 0;
		auto h = p.onFlush([&event](Frame &frame, Channel c) {
			event++;
			return true;
		});

		{
			Frame f = Feed::make(&p, FrameID(0,0), 0);
			f.store();
			f.create<int>(Channel::Pose) = 55;
			REQUIRE( event == 0 );
		}

		REQUIRE( event == 1 );
	}

	SECTION("no flush of flushed frame on destruct") {
		Session p;

		int event = 0;
		auto h = p.onFlush([&event](Frame &frame, Channel c) {
			event++;
			return true;
		});

		{
			Frame f = Feed::make(&p, FrameID(0,0), 0);
			f.store();
			f.create<int>(Channel::Pose) = 55;
			f.flush();
			REQUIRE( event == 1 );
		}

		REQUIRE( event == 1 );
	}
}

/* #4.2.1 */
TEST_CASE("ftl::data::Frame flush foreign", "[4.2.1]") {
	// For local flush see #4.1.1

	SECTION("event on foreign flush") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);

		f.createChange<int>(Channel::Colour, ChangeType::FOREIGN) = 55;
		f.store();

		int event = 0;
		auto h = f.onFlush([&event](Frame &frame, Channel c) {
			event++;
			return true;
		});

		REQUIRE( event == 0 );
		f.flush();
		REQUIRE( event == 1 );
	}
}

/* #4.2.2 */
TEST_CASE("ftl::data::Frame no flush of completed", "[4.2.2]") {
	// For local flush see #4.1.1

	SECTION("no event on completed flush") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);

		f.createChange<int>(Channel::Colour, ChangeType::COMPLETED) = 55;
		f.store();

		int event = 0;
		auto h = f.onFlush([&event](Frame &frame, Channel c) {
			event++;
			return true;
		});

		REQUIRE( event == 0 );
		f.flush();
		REQUIRE( event == 0 );
	}
}

/* #4.2.3 see #2.2.4 */

/* #4.3.1 see #4.2.1 */

/* #4.3.2 see #4.2.2 but also Feed class */

/* #4.3.3 see #2.2.4 */

/* #4.4.1 see #4.1.6 and #4.2.1 */

/* #4.4.2 see #4.2.2 */

/* #4.4.3 */
TEST_CASE("ftl::data::Frame parallel flush events", "[4.4.3]") {
	SECTION("event for each of multiple changes") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);
		f.store();

		int event = 0;
		auto h = f.onFlush([&event](Frame &frame, Channel c) {
			event++;
			return true;
		});

		f.create<int>(Channel::Pose);
		f.create<int>(Channel::Colour);
		f.create<int>(Channel::Depth);
		REQUIRE( event == 0 );

		f.flush();
		REQUIRE( event == 3 );
	}
}

/* #4.4.4 see #4.4.3 */

/* #4.4.5 */
TEST_CASE("ftl::data::Frame aggregate flush events", "[4.4.5]") {
	ftl::data::make_channel<std::list<int>>(Channel::Density, "density", ftl::data::StorageMode::AGGREGATE);

	SECTION("multiple changes cause single event") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);
		f.store();

		int event = 0;
		auto h = f.onFlush([&event](Frame &frame, Channel c) {
			event++;
			return true;
		});

		f.create<std::list<int>>(Channel::Density) = {34};
		f.create<std::list<int>>(Channel::Density) = {55};
		f.create<std::list<int>>(Channel::Density) = {12,89};
		REQUIRE( event == 0 );

		f.flush();
		REQUIRE( event == 1 );
	}

	ftl::data::clearRegistry();
}

/* #4.4.6 */
// TODO: Check function signature

/* #4.4.7 FIXME: Review specification */
TEST_CASE("ftl::data::Frame status after flush", "[4.4.7]") {
	SECTION("still changed after flush") {
		Session p;
		Frame f = Feed::make(&p, FrameID(0,0), 0);
		f.store();

		f.create<int>(Channel::Colour) = 55;
		f.flush();

		REQUIRE( f.changed(Channel::Colour) );
	}
}

/* #5 FIXME: RPC not implemented. */

/* #6 See pool unit tests */



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
bool ftl::data::make_type<TestC>() {
	return false;
}

template <>
TestA &ftl::data::Frame::create<TestA>(ftl::codecs::Channel c) {
	return create<TestC>(c).a;
}

template <>
TestB &ftl::data::Frame::create<TestB>(ftl::codecs::Channel c) {
	return create<TestC>(c).b;
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
	ftl::data::make_channel<TestC>(Channel::Pose, "pose", ftl::data::StorageMode::PERSISTENT);

	SECTION("Create and get first type with default") {
		Frame f = Feed::make(nullptr, FrameID(0,0), 0);
		f.store();
		f.create<TestA>(Channel::Pose);
		
		auto *x = f.getPtr<TestA>(Channel::Pose);
		REQUIRE( x );
		REQUIRE( x->a == 55 );

		auto *y = f.getPtr<TestB>(Channel::Pose);
		REQUIRE( y );
		REQUIRE( y->b == 99 );
	}

	SECTION("Create and get first type with value") {
		Frame f = Feed::make(nullptr, FrameID(0,0), 0);
		f.store();
		f.create<TestA>(Channel::Pose) = {77};
		
		auto *x = f.getPtr<TestA>(Channel::Pose);
		REQUIRE( x );
		REQUIRE( x->a == 77 );

		auto *y = f.getPtr<TestB>(Channel::Pose);
		REQUIRE( y );
		REQUIRE( y->b == 99 );
	}

	ftl::data::clearRegistry();
}
