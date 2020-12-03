/**
 * @file timer.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#ifndef _FTL_COMMON_TIMER_HPP_
#define _FTL_COMMON_TIMER_HPP_

#include <ftl/handle.hpp>
#include <functional>
#include <optional>

namespace ftl {

class UUID;

/**
 * A single global timer mechanism to call functions with either high or low
 * precision timing accuracy. This is used to provide accurate frame timing for
 * capture and rendering sync and it is deliberately a namespace and not a class
 * since the entire system should be operating at a single frame rate. It
 * controls the entire pipelines behaviour. It uses timestamps that are
 * multiples of the millisecond interval between frames.
 */
namespace timer {

/**
 * Timer level determines in what order and when a timer callback is called.
 * This allows some timers to operate at higher precision / lower latency
 * than others, as well as having idle callbacks.
 */
enum timerlevel_t {
	kTimerHighPrecision = 0,
	kTimerSwap,
	kTimerMain,
	kTimerIdle1,	// 1ms jobs to optionally do whilst idle
	kTimerIdle10,	// 10ms jobs to optionally do whilst idle
	kTimerMAXLEVEL
};

int64_t get_time();

/**
 * Milliseconds between calls.
 */
void setInterval(int ms);

/**
 * The highprec parameter sets whether or not this
 * timer should be high precision on the calling interval. A high precision
 * timer involves spinning the cpu to achieve millisecond accuracy.
 */
void setHighPrecision(bool hp);

int getInterval();

/**
 * Get current (monotonic) time in milliseconds.
 */
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
ftl::Handle add(timerlevel_t, const std::function<bool(int64_t ts)> &);

/**
 * Same as other add function except that a multiplier is given to indicate
 * how often this should be triggered in numbers of ticks.
 */
ftl::Handle add(timerlevel_t, size_t multiplier, const std::function<bool(int64_t ts)> &);

/**
 * Same as other add function except that a period in seconds is given. Note that
 * the period should be a multiple of frames otherwise it will not be accurate
 * but will still work.
 */
ftl::Handle add(timerlevel_t, double seconds, const std::function<bool(int64_t ts)> &);

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

void setTimeMaster(const ftl::UUID &m);

std::optional<ftl::UUID> getTimeMaster();

}
}

#endif  // _FTL_COMMON_TIMER_HPP_
