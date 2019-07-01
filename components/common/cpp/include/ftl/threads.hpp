#ifndef _FTL_THREADS_HPP_
#define _FTL_THREADS_HPP_

#include <mutex>
#include <shared_mutex>
#include <ctpl_stl.h>

#define POOL_SIZE 10

// #define DEBUG_MUTEX

#if defined DEBUG_MUTEX
#include <loguru.hpp>
#include <chrono>
#include <type_traits>

#define MUTEX std::timed_mutex
#define RECURSIVE_MUTEX std::recursive_timed_mutex
#define SHARED_MUTEX std::shared_timed_mutex

#define UNIQUE_LOCK(M,L) std::unique_lock<std::remove_reference<decltype(M)>::type> L(M, std::chrono::seconds(5)); if (!L) LOG(FATAL) << "Mutex deadlock";
#define SHARED_LOCK(M,L) std::shared_lock<std::remove_reference<decltype(M)>::type> L(M, std::chrono::seconds(5)); if (!L) LOG(FATAL) << "Mutex deadlock";

#else
#define MUTEX std::mutex
#define RECURSIVE_MUTEX std::recursive_mutex
#define SHARED_MUTEX std::shared_mutex

#define UNIQUE_LOCK(M,L) std::unique_lock<std::remove_reference<decltype(M)>::type> L(M);
#define SHARED_LOCK(M,L) std::shared_lock<std::remove_reference<decltype(M)>::type> L(M);
#endif  // DEBUG_MUTEX

namespace ftl {
	extern ctpl::thread_pool pool;
}

#endif  // _FTL_THREADS_HPP_
