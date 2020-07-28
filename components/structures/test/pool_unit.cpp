/*
 * These tests directly relate to the specification found at:
 *     https://gitlab.utu.fi/nicolas.pope/ftl/-/wikis/Design/Frames
 * 
 * Starting from section 5 on memory management.
 */

#include "catch.hpp"

#include <ftl/data/framepool.hpp>

using ftl::data::Session;
using ftl::data::Frame;
using ftl::data::Pool;
using ftl::codecs::Channel;
using ftl::data::ChangeType;
using ftl::data::StorageMode;
using ftl::data::FrameStatus;
using ftl::data::FrameID;

/* #5.1 */
TEST_CASE("ftl::data::Pool create frames", "[5.1]") {
	SECTION("can allocate valid frame from pool") {
		Pool pool(5,5);

		Frame f = pool.allocate(ftl::data::FrameID(0,2), 100);
		REQUIRE( f.status() == FrameStatus::CREATED );
		REQUIRE( pool.size() == 4 );
		REQUIRE( f.source() == 2 );
		REQUIRE( f.timestamp() == 100 );
	}
}

/* #5.2 */
TEST_CASE("ftl::data::Pool release frames on destruct", "[5.1]") {
	SECTION("can destroy allocated frame") {
		Pool pool(5,5);

		{
			Frame f = pool.allocate(ftl::data::FrameID(0,0), 0);
			REQUIRE( f.status() == FrameStatus::CREATED );
			REQUIRE( pool.size() == 4 );
		}

		REQUIRE( pool.size() == 5 );
	}

	SECTION("data reused between allocations") {
		Pool pool(1,1);

		const int *ptr = nullptr;

		{
			Frame f = pool.allocate(ftl::data::FrameID(0,0), 0);
			f.store();
			f.create<std::vector<int>>(Channel::Colour) = {44,55,66};
			ptr = f.get<std::vector<int>>(Channel::Colour).data();
		}

		REQUIRE( pool.size() == 1 );

		{
			Frame f = pool.allocate(ftl::data::FrameID(0,0), 0);
			f.store();
			auto &v = f.create<std::vector<int>>(Channel::Colour);

			REQUIRE( v[0] == 44 );
			REQUIRE( v[1] == 55 );
			REQUIRE( v[2] == 66 );

			REQUIRE( (ptr && ptr == v.data()) );
		}
	}
}

/* #5.3 */
TEST_CASE("ftl::data::Pool reused frames are stale", "[5.3]") {
	SECTION("data reused is stale") {
		Pool pool(1,1);

		{
			Frame f = pool.allocate(ftl::data::FrameID(0,0), 0);
			f.store();
			f.create<std::vector<int>>(Channel::Colour) = {44,55,66};
		}

		REQUIRE( pool.size() == 1 );

		{
			Frame f = pool.allocate(ftl::data::FrameID(0,0), 0);
			f.store();

			REQUIRE( !f.has(Channel::Colour) );
			REQUIRE( !f.changed(Channel::Colour) );

			auto &v = f.create<std::vector<int>>(Channel::Colour);
			REQUIRE( v[0] == 44 );
		}
	}
}

/* #5.4 */
// Hard to test

/* #5.5 */
TEST_CASE("ftl::data::Pool excessive allocations", "[5.5]") {
	SECTION("allocate far beyond pool size") {
		Pool pool(10,20);

		{
			std::list<Frame> l;
			for (int i=0; i<100; ++i) {
				l.push_back(std::move(pool.allocate(FrameID(0,0),0)));
			}

			REQUIRE( pool.size() >= 10 );
		}

		// 2*pool size is the chosen max
		REQUIRE( pool.size() <= 20 );
	}
}

TEST_CASE("ftl::data::Pool persistent sessions", "[]") {
	SECTION("persistent across timetstamps") {
		Pool pool(10,20);

		{
			Frame f = pool.allocate(ftl::data::FrameID(0,0), 10);
			f.store();
			f.create<int>(Channel::Pose) = 567;
		}

		REQUIRE( (pool.session(FrameID(0,0)).get<int>(Channel::Pose) == 567) );

		{
			Frame f = pool.allocate(ftl::data::FrameID(0,0), 20);
			f.store();
			REQUIRE( f.get<int>(Channel::Pose) == 567 );
		}
	}

	SECTION("persistent across many timetstamps") {
		Pool pool(10,20);

		{
			Frame f = pool.allocate(ftl::data::FrameID(0,0), 10);
			f.store();
			f.create<int>(Channel::Pose) = 567;
		}

		REQUIRE( (pool.session(FrameID(0,0)).get<int>(Channel::Pose) == 567) );

		{
			Frame f = pool.allocate(ftl::data::FrameID(0,0), 20);
			f.store();
			REQUIRE( f.get<int>(Channel::Pose) == 567 );
		}

		{
			Frame f = pool.allocate(ftl::data::FrameID(0,0), 30);
			f.store();
			REQUIRE( f.get<int>(Channel::Pose) == 567 );
		}
	}

	SECTION("persistent across frames and timetstamps") {
		Pool pool(10,20);

		{
			Frame f = pool.allocate(ftl::data::FrameID(0,0), 10);
			f.store();
			f.create<int>(Channel::Pose) = 567;
		}

		{
			Frame f = pool.allocate(ftl::data::FrameID(0,1), 10);
			f.store();
			f.create<int>(Channel::Pose) = 568;
		}

		REQUIRE( (pool.session(FrameID(0,0)).get<int>(Channel::Pose) == 567) );

		{
			Frame f = pool.allocate(ftl::data::FrameID(0,0), 20);
			f.store();
			REQUIRE( f.get<int>(Channel::Pose) == 567 );
		}

		{
			Frame f = pool.allocate(ftl::data::FrameID(0,1), 20);
			f.store();
			REQUIRE( f.get<int>(Channel::Pose) == 568 );
		}
	}

	SECTION("persistent across framesets and timetstamps") {
		Pool pool(10,20);

		{
			Frame f = pool.allocate(ftl::data::FrameID(0,0), 10);
			f.store();
			f.create<int>(Channel::Pose) = 567;
		}

		{
			Frame f = pool.allocate(ftl::data::FrameID(1,0), 10);
			f.store();
			f.create<int>(Channel::Pose) = 568;
		}

		REQUIRE( (pool.session(FrameID(0,0)).get<int>(Channel::Pose) == 567) );

		{
			Frame f = pool.allocate(ftl::data::FrameID(0,0), 20);
			f.store();
			REQUIRE( f.get<int>(Channel::Pose) == 567 );
		}

		{
			Frame f = pool.allocate(ftl::data::FrameID(1,0), 20);
			f.store();
			REQUIRE( f.get<int>(Channel::Pose) == 568 );
		}
	}	
}

