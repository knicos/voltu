#ifndef _FTL_COMMON_TIMER_HPP_
#define _FTL_COMMON_TIMER_HPP_

#include <functional>

namespace ftl {

/**
 * A single global timer mechanism to call functions with either high or low
 * precision timing accuracy. This is used to provide accurate frame timing for
 * capture and rendering sync and it is deliberately a namespace and not a class
 * since the entire system should be operating at a single frame rate. It
 * controls the entire pipelines behaviour. It uses timestamps that are
 * multiples of the millisecond interval between frames.
 */
namespace timer {

enum timerlevel_t {
	kTimerHighPrecision = 0,
	kTimerSwap,
	kTimerMain,
	kTimerIdle1,	// 1ms jobs to optionally do whilst idle
	kTimerIdle10,	// 10ms jobs to optionally do whilst idle
	kTimerMAXLEVEL
};

/**
 * Represents a timer job for control purposes. Use to remove timer jobs in
 * a destructor, for example.
 */
struct TimerHandle {
	TimerHandle() : id_(-1) {}
	explicit TimerHandle(int i) : id_(i) {}
	TimerHandle(const TimerHandle &t) : id_(t.id()) {}

	/**
	 * Cancel the timer job. If currently executing it will block and wait for
	 * the job to complete.
	 */
	void cancel() const;
	void pause() const;
	void unpause() const;

	/**
	 * Do the timer job every N frames.
	 */
	void setMultiplier(unsigned int) const;

	/**
	 * Give the timer job a name for logging output.
	 */
	void setName(const std::string &) const;

	/**
	 * Allow copy assignment.
	 */
	TimerHandle &operator=(const TimerHandle &h) { id_ = h.id(); return *this; }

	inline int id() const { return id_; }

	private:
	int id_;
};

int64_t get_time();

/**
 * Milliseconds between calls.
 */
void setInterval(int ms);

int getInterval();

int64_t get_time();
int64_t get_time_micro();
double get_time_seconds();

/**
 * Add the specified number of milliseconds to the clock when generating
 * timestamps. This is used to synchronise clocks on multiple machines as it
 * influences the behaviour of the timer.
 */
void setClockAdjustment(int64_t ms);

/**
 * Whether or not this machine should sync its clocks to someone else. Should
 * be true on capture / vision nodes and false on a reconstruction node.
 */
bool isClockSlave();

/**
 * Change clock slave mode. If set to true then this machine will attempt to
 * adjust its clock to another machines.
 */
void setClockSlave(bool);

/**
 * Add a timer callback with a given precision and ordering. The highest
 * precision callbacks occur within 1ms of required time and should return
 * almost immediately to prevent delays to other callbacks. Other precisions
 * occur later and in separate thread jobs for each callback. If a callback
 * fails to return before the next time step, it is skipped for that timestep.
 * If all high precision callbacks together take more than 1ms to complete, a
 * warning is produced.
 */
const TimerHandle add(timerlevel_t, const std::function<bool(int64_t ts)> &);

/**
 * Initiate the timer and optionally block the current process.
 */
void start(bool block=false);

/**
 * Stop the timer after any current callbacks complete. Blocks until stopped.
 */
void stop(bool wait=true);

size_t count(timerlevel_t);

/**
 * Stop and clear all callbacks. Used for testing purposes.
 */
void reset();

}
}

#endif  // _FTL_COMMON_TIMER_HPP_
