#include <ftl/timer.hpp>
#include <ftl/threads.hpp>
#include <loguru.hpp>

#include <vector>
#include <list>
#include <chrono>

#include <xmmintrin.h>

using std::vector;
using std::list;
using std::function;
using std::chrono::time_point_cast;
using std::chrono::milliseconds;
using std::chrono::high_resolution_clock;
using std::this_thread::sleep_for;

using namespace ftl::timer;

static int64_t last_frame = 0;
static int64_t mspf = 50;
static int64_t clock_adjust = 0;
static bool active = false;
static std::atomic<int> active_jobs = 0;
static MUTEX mtx;
static int last_id = 0;

struct TimerJob {
	int id;
	function<bool(int64_t)> job;
	volatile bool active;
	bool paused;
	int multiplier;
	int countdown;
	std::string name;
};

static list<TimerJob> jobs[kTimerMAXLEVEL];

int64_t ftl::timer::get_time() {
	return time_point_cast<milliseconds>(high_resolution_clock::now()).time_since_epoch().count()+clock_adjust;
}

static void waitTimePoint() {
	auto start = high_resolution_clock::now();
	int64_t now = get_time();
	int64_t target = now / mspf;
	int64_t msdelay = mspf - (now % mspf);
	int64_t sincelast = now - last_frame*mspf;

	if (sincelast > mspf) LOG(WARNING) << "Frame " << "(" << (target-last_frame) << ") dropped by " << sincelast << "ms";

	// Use sleep_for for larger delays
	
	//LOG(INFO) << "Required Delay: " << (now / 40) << " = " << msdelay;
	while (msdelay >= 35 && sincelast != mspf) {
		sleep_for(milliseconds(10));
		now = get_time();
		msdelay = mspf - (now % mspf);
	}

	// Still lots of time so do some idle jobs
	if (msdelay >= 10 && sincelast != mspf) {
		UNIQUE_LOCK(mtx, lk);
		auto idle_job = jobs[kTimerIdle10].begin();
		while (idle_job != jobs[kTimerIdle10].end() && msdelay >= 10 && sincelast != mspf) {
			(*idle_job).active = true;
			bool doremove = !(*idle_job).job(now);

			if (doremove) {
				idle_job = jobs[kTimerIdle10].erase(idle_job);
				LOG(INFO) << "Timer job removed";
			} else {
				(*idle_job++).active = false;
			}
			now = get_time();
			msdelay = mspf - (now % mspf);
		}
	}

	// Spin loop until exact grab time
	//LOG(INFO) << "Spin Delay: " << (now / 40) << " = " << (40 - (now%40));

	if (sincelast != mspf) {
		target = now / mspf;
		while ((now/mspf) == target) {
			_mm_pause();  // SSE2 nano pause intrinsic
			now = get_time();
		};
	}
	last_frame = now/mspf;
}

void ftl::timer::setInterval(int ms) {
	mspf = ms;
}

int ftl::timer::getInterval() {
	return mspf;
}

void ftl::timer::setClockAdjustment(int64_t ms) {
	clock_adjust += ms;
}

const TimerHandle ftl::timer::add(timerlevel_t l, const std::function<bool(int64_t ts)> &f) {
	if (l < 0 || l >= kTimerMAXLEVEL) return {};

	UNIQUE_LOCK(mtx, lk);
	int newid = last_id++;
	jobs[l].push_back({newid, f, false, false, 0, 0, "NoName"});
	return TimerHandle(newid);
}

static void removeJob(int id) {
	UNIQUE_LOCK(mtx, lk);
	if (id < 0) return;
	for (size_t j=0; j<kTimerMAXLEVEL; ++j) {
		for (auto i=jobs[j].begin(); i!=jobs[j].end(); i++) {
			if ((*i).id == id) {
				while ((*i).active) {
					sleep_for(milliseconds(10));
				}

				jobs[j].erase(i);
				return;
			}
		}
	}
}

static void trigger_jobs() {
	UNIQUE_LOCK(mtx, lk);
	const int64_t ts = last_frame*mspf;

	// First do non-blocking high precision callbacks
	const int64_t before = get_time();
	for (auto &j : jobs[kTimerHighPrecision]) {
		j.job(ts);
	}
	const int64_t after = get_time();
	if (after - before > 0) LOG(WARNING) << "Precision jobs took too long (" << (after-before) << "ms)";

	// Then do also non-blocking swap callbacks
	for (auto &j : jobs[kTimerSwap]) {
		j.job(ts);
	}

	// Now use thread jobs to do more intensive callbacks
	for (auto &j : jobs[kTimerMain]) {
		if (j.active) {
			//LOG(WARNING) << "Timer job too slow ... skipped for " << ts;
			continue;
		}
		j.active = true;
		active_jobs++;
		ftl::pool.push([&j,ts](int id) {
			bool doremove = !j.job(ts);
			j.active = false;
			active_jobs--;
			if (doremove) removeJob(j.id);
		});
	}
}

namespace ftl {
	extern bool running;
}

void ftl::timer::start(bool block) {
	if (active) return;
	active = true;

	if (block) {
		active_jobs++;
		while (ftl::running && active) {
			waitTimePoint();
			trigger_jobs();
		}
		active_jobs--;
	} else {
		ftl::pool.push([](int id) {
			active_jobs++;
			while (ftl::running && active) {
				waitTimePoint();
				trigger_jobs();
			}
			active_jobs--;
		});
	}
}

void ftl::timer::stop(bool wait) {
	active = false;

	if (wait) {
		// All callbacks must complete before returning.
		while (active_jobs > 0) {
			sleep_for(milliseconds(10));
		}
	}
}

size_t ftl::timer::count(ftl::timer::timerlevel_t l) {
	if (l < 0 || l >= kTimerMAXLEVEL) return 0;
	return jobs[l].size();
}

void ftl::timer::reset() {
	stop(true);
	for (int i=0; i<ftl::timer::kTimerMAXLEVEL; i++) {
		jobs[i].clear();
	}
}

// ===== TimerHandle ===========================================================

void ftl::timer::TimerHandle::cancel() const {
	removeJob(id());
}

void ftl::timer::TimerHandle::pause() const {

}

void ftl::timer::TimerHandle::unpause() const {

}

void ftl::timer::TimerHandle::setMultiplier(unsigned int N) const {

}

void ftl::timer::TimerHandle::setName(const std::string &name) const {

}
