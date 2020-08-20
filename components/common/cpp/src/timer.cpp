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
static bool hprec_ = false;
static int64_t clock_adjust = 0;
static bool active = false;
static std::atomic<int> active_jobs = 0;
static MUTEX mtx;
static int last_id = 0;
static bool clock_slave = true;
static std::future<void> timer_future;

struct TimerJob {
	int id=0;
	ftl::SingletonHandler<int64_t> job;
	std::atomic_bool active=false;
	// TODO: (Nick) Implement richer forms of timer
	//bool paused;
	int multiplier=0;		// Number of ticks before trigger
	int counter=0;		// Current tick counter
	std::string name;
};

static list<TimerJob> jobs[kTimerMAXLEVEL];

int64_t ftl::timer::get_time() {
	return time_point_cast<milliseconds>(high_resolution_clock::now()).time_since_epoch().count()+clock_adjust;
}

int64_t ftl::timer::get_time_micro() {
	return time_point_cast<std::chrono::microseconds>(high_resolution_clock::now()).time_since_epoch().count()+(clock_adjust*1000);
}

double ftl::timer::get_time_seconds() {
	return time_point_cast<std::chrono::microseconds>(high_resolution_clock::now()).time_since_epoch().count() / 1000000.0 + (static_cast<double>(clock_adjust) / 1000.0);
}

static void waitTimePoint() {
	int64_t now = get_time();
	int64_t target = now / mspf;
	int64_t msdelay = mspf - (now % mspf);
	int64_t sincelast = now - last_frame*mspf;

	if (hprec_ && sincelast > mspf) LOG(WARNING) << "Frame " << "(" << (target-last_frame) << ") dropped by " << (sincelast-mspf) << "ms";

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
			auto &job = *idle_job;

			if (++job.counter >= job.multiplier) {
				job.counter = 0;
				job.active = true;
				bool doremove = !job.job.trigger(now);

				if (doremove) {
					idle_job = jobs[kTimerIdle10].erase(idle_job);
					LOG(INFO) << "Timer job removed";
				} else {
					(*idle_job++).active = false;
				}
			} else {
				++idle_job;
			}
			now = get_time();
			msdelay = mspf - (now % mspf);
		}
	}

	/*while (msdelay >= 10 && sincelast != mspf) {
		sleep_for(milliseconds(5));
		now = get_time();
		msdelay = mspf - (now % mspf);
	}*/

	if (msdelay >= 2 && sincelast != mspf) {
		UNIQUE_LOCK(mtx, lk);
		auto idle_job = jobs[kTimerIdle1].begin();
		while (idle_job != jobs[kTimerIdle1].end() && msdelay >= 2 && sincelast != mspf) {
			auto &job = *idle_job;

			if (++job.counter >= job.multiplier) {
				job.counter = 0;
				job.active = true;
				bool doremove = !job.job.trigger(now);

				if (doremove) {
					idle_job = jobs[kTimerIdle1].erase(idle_job);
					LOG(INFO) << "Timer job removed";
				} else {
					(*idle_job++).active = false;
				}
			} else {
				++idle_job;
			}
			now = get_time();
			msdelay = mspf - (now % mspf);
		}
	}

	if (hprec_) {
		// Spin loop until exact grab time
		// Accurate to around 4 micro seconds.
		if (sincelast != mspf) {
			// TODO: Try using sleep_for for msdelay-1
			target = now / mspf;
			while ((now/mspf) == target) {
				_mm_pause();  // SSE2 nano pause intrinsic
				now = get_time();
			};
		}
	} else {
		// Accurate to just under 1 millisecond usually
		if (sincelast != mspf) sleep_for(milliseconds(msdelay));
		now = get_time();
	}
	last_frame = now/mspf;
	int64_t over = now - (last_frame*mspf);
	if (over > 1) LOG(WARNING) << "Timer off by " << over << "ms";
}

void ftl::timer::setInterval(int ms) {
	mspf = ms;
}

void ftl::timer::setHighPrecision(bool hp) {
	hprec_ = hp;
}

int ftl::timer::getInterval() {
	return static_cast<int>(mspf);
}

void ftl::timer::setClockAdjustment(int64_t ms) {
	clock_adjust += ms;
}

void ftl::timer::setClockSlave(bool s) {
	clock_slave = s;
}

bool ftl::timer::isClockSlave() {
	return clock_slave;
}

ftl::Handle ftl::timer::add(timerlevel_t l, const std::function<bool(int64_t ts)> &f) {
	if (l < 0 || l >= kTimerMAXLEVEL) return {};

	UNIQUE_LOCK(mtx, lk);
	int newid = last_id++;
	auto &j = jobs[l].emplace_back();
	j.id = newid;
	j.name = "NoName";
	ftl::Handle h = j.job.on(f);
	return h;
}

ftl::Handle ftl::timer::add(timerlevel_t l, size_t multiplier, const std::function<bool(int64_t ts)> &f) {
	if (l < 0 || l >= kTimerMAXLEVEL) return {};

	UNIQUE_LOCK(mtx, lk);
	int newid = last_id++;
	auto &j = jobs[l].emplace_back();
	j.id = newid;
	j.name = "NoName";
	j.multiplier = multiplier;
	ftl::Handle h = j.job.on(f);
	return h;
}

ftl::Handle ftl::timer::add(timerlevel_t l, double seconds, const std::function<bool(int64_t ts)> &f) {
	if (l < 0 || l >= kTimerMAXLEVEL) return {};

	UNIQUE_LOCK(mtx, lk);
	int newid = last_id++;
	auto &j = jobs[l].emplace_back();
	j.id = newid;
	j.name = "NoName";
	j.multiplier = int(seconds*1000.0 / double(getInterval()));
	ftl::Handle h = j.job.on(f);
	return h;
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

	if (active_jobs > 1) {
		LOG(WARNING) << "Previous timer incomplete, skipping " << ts;
		return;
	}

	// First do non-blocking high precision callbacks
	const int64_t before = get_time();
	for (auto &j : jobs[kTimerHighPrecision]) {
		j.job.trigger(ts);
	}
	const int64_t after = get_time();
	if (after - before > 1) LOG(WARNING) << "Precision jobs took too long (" << (after-before) << "ms)";

	// Then do also non-blocking swap callbacks
	for (auto &j : jobs[kTimerSwap]) {
		j.job.trigger(ts);
	}

	// Now use thread jobs to do more intensive callbacks
	for (auto &j : jobs[kTimerMain]) {
		if (j.active) {
			LOG(WARNING) << "Timer job too slow ... skipped for " << ts;
			continue;
		}
		j.active = true;
		active_jobs++;

		auto *pj = &j;

		// If last job in list then do in this thread
		if (active_jobs == static_cast<int>(jobs[kTimerMain].size())+1) {
			lk.unlock();
			bool doremove = true;
			try {
				doremove = !pj->job.trigger(ts);
			} catch(const std::exception &e) {
				LOG(ERROR) << "Exception in timer job: " << e.what();
			}
			pj->active = false;
			active_jobs--;
			if (doremove) removeJob(pj->id);
			lk.lock();
			break;
		} else {
			ftl::pool.push([pj,ts](int id) {
				bool doremove = true;
				try {
					doremove = !pj->job.trigger(ts);
				} catch(const std::exception &e) {
					LOG(ERROR) << "Exception in timer job: " << e.what();
				}
				pj->active = false;
				active_jobs--;
				if (doremove) removeJob(pj->id);
			});
		}
	}

	// Final cleanup of stale jobs
	for (size_t j=0; j<kTimerMAXLEVEL; ++j) {
		for (auto i=jobs[j].begin(); i!=jobs[j].end(); i++) {
			if ((bool)((*i).job) == false) {
				i = jobs[j].erase(i);
			}
		}
	}
}

namespace ftl {
	extern bool running;
}

//static MUTEX mtx;

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
		timer_future = ftl::pool.push([](int id) {
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
		try {
			if (timer_future.valid()) timer_future.get();
		} catch (const std::exception &e) {
			LOG(ERROR) << "Timer exception: " << e.what();
		}

		int attempts = 10;

		// All callbacks must complete before returning.
		while (active_jobs > 0 && attempts-- > 0) {
			sleep_for(milliseconds(10));
		}

		if (active_jobs > 0) LOG(WARNING) << "Forced job stop: " << active_jobs;
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
