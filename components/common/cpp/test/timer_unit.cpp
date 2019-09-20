#include "catch.hpp"
#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>
#include <ftl/timer.hpp>
#include <ftl/threads.hpp>

ctpl::thread_pool ftl::pool(4);

namespace ftl {
	bool running = true;
}

TEST_CASE( "Timer::add() High Precision Accuracy" ) {
	SECTION( "An instantly returning callback" ) {
		bool didrun = false;

		ftl::timer::reset();

		auto rc = ftl::timer::add(ftl::timer::kTimerHighPrecision, [&didrun](int64_t ts) {
			didrun = true;
			ftl::timer::stop(false);
			return true;
		});

		REQUIRE( (rc.id() >= 0) );

		ftl::timer::start(true);
		REQUIRE( didrun == true );
	}

	SECTION( "A slow returning callback" ) {
		bool didrun = false;

		ftl::timer::reset();

		auto rc = ftl::timer::add(ftl::timer::kTimerHighPrecision, [&didrun](int64_t ts) {
			didrun = true;
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
			ftl::timer::stop(false);
			return true;
		});

		REQUIRE( (rc.id() >= 0) );

		ftl::timer::start(true);
		REQUIRE( didrun == true );
	}

	SECTION( "Multiple callback" ) {
		bool didrun[3] = {false};

		ftl::timer::reset();

		auto rc = ftl::timer::add(ftl::timer::kTimerHighPrecision, [&didrun](int64_t ts) {
			didrun[0] = true;
			ftl::timer::stop(false);
			return true;
		});

		REQUIRE( (rc.id() >= 0) );

		ftl::timer::add(ftl::timer::kTimerHighPrecision, [&didrun](int64_t ts) {
			didrun[1] = true;
			ftl::timer::stop(false);
			return true;
		});

		ftl::timer::add(ftl::timer::kTimerHighPrecision, [&didrun](int64_t ts) {
			didrun[2] = true;
			ftl::timer::stop(false);
			return true;
		});

		ftl::timer::start(true);
		REQUIRE( didrun[0] == true );
		REQUIRE( didrun[1] == true );
		REQUIRE( didrun[2] == true );
	}
}

TEST_CASE( "Timer::add() Idle10 job" ) {
	SECTION( "Quick idle job" ) {
		bool didrun = false;

		ftl::timer::reset();

		auto rc = ftl::timer::add(ftl::timer::kTimerIdle10, [&didrun](int64_t ts) {
			didrun = true;
			ftl::timer::stop(false);
			return true;
		});

		REQUIRE( (rc.id() >= 0) );

		ftl::timer::start(true);
		REQUIRE( didrun == true );
	}

	SECTION( "Slow idle job" ) {
		bool didrun = false;

		ftl::timer::reset();

		auto rc = ftl::timer::add(ftl::timer::kTimerIdle10, [&didrun](int64_t ts) {
			didrun = true;
			std::this_thread::sleep_for(std::chrono::milliseconds(60));
			ftl::timer::stop(false);
			return true;
		});

		REQUIRE( (rc.id() >= 0) );

		ftl::timer::start(true);
		REQUIRE( didrun == true );
	}

	SECTION( "Return remove idle job" ) {
		bool didrun = false;

		ftl::timer::reset();

		auto rc = ftl::timer::add(ftl::timer::kTimerIdle10, [&didrun](int64_t ts) {
			didrun = true;
			ftl::timer::stop(false);
			return false;
		});

		REQUIRE( (rc.id() >= 0) );

		ftl::timer::start(true);
		REQUIRE( didrun == true );
		REQUIRE( ftl::timer::count(ftl::timer::kTimerIdle10) == 0 );
	}
}

TEST_CASE( "Timer::add() Main job" ) {
	SECTION( "Quick main job" ) {
		bool didrun = false;

		ftl::timer::reset();

		auto rc = ftl::timer::add(ftl::timer::kTimerMain, [&didrun](int64_t ts) {
			didrun = true;
			ftl::timer::stop(false);
			return true;
		});

		REQUIRE( (rc.id() >= 0) );

		ftl::timer::start(true);
		REQUIRE( didrun == true );
	}

	SECTION( "Slow main job" ) {
		bool didrun = false;

		ftl::timer::reset();

		auto rc = ftl::timer::add(ftl::timer::kTimerMain, [&didrun](int64_t ts) {
			didrun = true;
			std::this_thread::sleep_for(std::chrono::milliseconds(60));
			ftl::timer::stop(false);
			return true;
		});

		REQUIRE( (rc.id() >= 0) );

		ftl::timer::start(true);
		REQUIRE( didrun == true );
	}

	SECTION( "Slow and fast main jobs" ) {
		int job1 = 0;
		int job2 = 0;

		ftl::timer::reset();

		auto rc = ftl::timer::add(ftl::timer::kTimerMain, [&job1](int64_t ts) {
			job1++;
			std::this_thread::sleep_for(std::chrono::milliseconds(60));
			ftl::timer::stop(false);
			return true;
		});

		REQUIRE( (rc.id() >= 0) );

		ftl::timer::add(ftl::timer::kTimerMain, [&job2](int64_t ts) {
			job2++;
			return true;
		});

		ftl::timer::start(true);
		REQUIRE( (job1 == 1 && job2 == 2) );
	}

	SECTION( "Return remove main job" ) {
		bool didrun = false;

		ftl::timer::reset();

		auto rc = ftl::timer::add(ftl::timer::kTimerMain, [&didrun](int64_t ts) {
			didrun = true;
			ftl::timer::stop(false);
			return false;
		});

		REQUIRE( (rc.id() >= 0) );

		ftl::timer::start(true);
		REQUIRE( didrun == true );
		REQUIRE( ftl::timer::count(ftl::timer::kTimerMain) == 0 );
	}
}

TEST_CASE( "TimerHandle::cancel()" ) {
	SECTION( "Invalid id" ) {
		bool didjob = false;
		ftl::timer::reset();

		ftl::timer::add(ftl::timer::kTimerMain, [&didjob](int64_t ts) {
			didjob = true;
			ftl::timer::stop(false);
			return true;
		});

		// Fake Handle
		ftl::timer::TimerHandle h(44);
		h.cancel();
		ftl::timer::start(true);
		REQUIRE( didjob );
	}

	SECTION( "Valid id" ) {
		bool didjob = false;
		ftl::timer::reset();

		auto id = ftl::timer::add(ftl::timer::kTimerMain, [&didjob](int64_t ts) {
			didjob = true;
			ftl::timer::stop(false);
			return true;
		});

		id.cancel();
		ftl::timer::start(false);
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		ftl::timer::stop();
		REQUIRE( !didjob );
	}
}
