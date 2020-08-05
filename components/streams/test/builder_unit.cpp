#include "catch.hpp"

#include <ftl/streams/builder.hpp>

using ftl::data::Pool;
using ftl::data::Frame;
using ftl::data::FrameSet;
using ftl::streams::ForeignBuilder;
using ftl::streams::LocalBuilder;
using ftl::codecs::Channel;

TEST_CASE("ftl::streams::ForeignBuilder can obtain a frameset", "[]") {
	SECTION("with one frame allocated") {
		Pool pool(2,5);
		ForeignBuilder builder(&pool, 44);

		builder.get(100, 0);
		{
			auto fs = builder.get(100);

			REQUIRE( fs->frameset() == 44 );
			REQUIRE( fs->source() == 255);
			REQUIRE( fs->timestamp() == 100 );
			REQUIRE( fs->frames.size() == 1 );
			REQUIRE( fs->frames[0].status() == ftl::data::FrameStatus::CREATED );
			REQUIRE( fs->frames[0].id() == (44<<8) );
			REQUIRE( fs->frames[0].timestamp() == 100 );
		}
	}

	SECTION("with five frames allocated") {
		Pool pool(2,5);
		ForeignBuilder builder(&pool, 44);

		builder.get(100, 4);
		builder.get(100, 0);

		{
			auto fs = builder.get(100);

			REQUIRE( fs->frameset() == 44 );
			REQUIRE( fs->timestamp() == 100 );
			REQUIRE( fs->frames.size() == 5 );
			REQUIRE( fs->frames[3].status() == ftl::data::FrameStatus::CREATED );
			REQUIRE( fs->frames[3].id() == (44<<8)+3 );
			REQUIRE( fs->frames[3].timestamp() == 100 );
		}
	}
}

TEST_CASE("ftl::streams::ForeignBuilder can complete a frame", "[]") {
	SECTION("with two frames allocated") {
		Pool pool(2,5);
		ForeignBuilder builder(&pool, 44);

		builder.get(100, 1);
		builder.get(100, 0);

		{
			auto fs = builder.get(100);
			fs->completed(0);

			REQUIRE( fs->frameset() == 44 );
			REQUIRE( fs->timestamp() == 100 );
			REQUIRE( fs->frames.size() == 2 );
			//REQUIRE( fs->frames[0].status() == ftl::data::FrameStatus::CREATED );
			REQUIRE( fs->firstFrame().id() == (44<<8) );
			REQUIRE( fs->firstFrame().timestamp() == 100 );
		}
	}
}

TEST_CASE("ftl::streams::ForeignBuilder can complete a frameset", "[]") {
	SECTION("with one frame allocated and no buffering") {
		Pool pool(2,5);
		ForeignBuilder builder(&pool, 44);

		builder.setBufferSize(0);

		builder.get(100, 0);

		int fsid = 0;

		auto h = builder.onFrameSet([&fsid](const ftl::data::FrameSetPtr& fs) {
			fsid = fs->frameset();
			return false;
		});

		{
			auto fs = builder.get(100);
			fs->completed(0);
		}

		// TODO: Find better way to wait...
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		REQUIRE( fsid == 44 );
	}

	SECTION("with two frames allocated and no buffering") {
		Pool pool(2,5);
		ForeignBuilder builder(&pool, 44);

		builder.setBufferSize(0);

		builder.get(100, 1);

		int fsid = 0;

		auto h = builder.onFrameSet([&fsid](const ftl::data::FrameSetPtr& fs) {
			fsid = fs->frameset();
			return false;
		});

		{
			auto fs = builder.get(100);
			fs->completed(0);
			fs->completed(1);
		}

		// TODO: Find better way to wait...
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		REQUIRE( fsid == 44 );
	}

	SECTION("does not complete a partial") {
		Pool pool(2,5);
		ForeignBuilder builder(&pool, 44);

		builder.setBufferSize(0);

		builder.get(100, 1);

		int fsid = 0;

		auto h = builder.onFrameSet([&fsid](const ftl::data::FrameSetPtr& fs) {
			fsid = fs->frameset();
			return false;
		});

		{
			auto fs = builder.get(100);
			fs->completed(1);
		}

		// TODO: Find better way to wait...
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		REQUIRE( fsid == 0 );
	}
}

TEST_CASE("ftl::streams::LocalBuilder can provide empty frames", "[]") {
	SECTION("a single empty frameset") {
		Pool pool(2,5);
		LocalBuilder builder(&pool, 45);

		auto fs = builder.getNextFrameSet(100);

		REQUIRE( fs );
		REQUIRE( fs->timestamp() == 100 );
		REQUIRE( fs->frames.size() == 1 );
		REQUIRE( fs->hasFrame(0) );
		REQUIRE( fs->mask != 0 );
	}

	SECTION("multiple framesets frameset") {
		Pool pool(2,5);
		LocalBuilder builder(&pool, 45);

		auto fs = builder.getNextFrameSet(100);
		fs->firstFrame().create<int>(Channel::Control) = 77;

		fs = builder.getNextFrameSet(110);

		REQUIRE( fs );
		REQUIRE( fs->timestamp() == 110 );
		REQUIRE( fs->frames.size() == 1 );
		REQUIRE( fs->hasFrame(0) );
		REQUIRE( fs->hasChannel(Channel::Control) == false );
	}
}

TEST_CASE("ftl::streams::LocalBuilder can provide filled frames", "[]") {
	SECTION("a single filled frameset") {
		Pool pool(2,5);
		LocalBuilder builder(&pool, 45);

		// Fake some received data, as done by Receiver class.
		{
			auto pfs = builder.get(100);
			pfs->firstFrame().createChange<int>(Channel::Control, ftl::data::ChangeType::FOREIGN) = 56;
			pfs->completed(0);
		}

		auto fs = builder.getNextFrameSet(100);

		REQUIRE( fs );
		REQUIRE( fs->timestamp() == 100 );
		REQUIRE( fs->frames.size() == 1 );
		REQUIRE( fs->frames[0].get<int>(Channel::Control) == 56 );
	}
}
