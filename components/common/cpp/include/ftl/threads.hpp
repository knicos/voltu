#ifndef _FTL_THREADS_HPP_
#define _FTL_THREADS_HPP_

#include <mutex>
#include <shared_mutex>
#include <ctpl_stl.h>

#define POOL_SIZE 10

//#define DEBUG_MUTEX
#define MUTEX_TIMEOUT 2

#if defined DEBUG_MUTEX
#include <loguru.hpp>
#include <chrono>
#include <type_traits>

#define MUTEX std::timed_mutex
#define RECURSIVE_MUTEX std::recursive_timed_mutex
#define SHARED_MUTEX std::shared_timed_mutex

#define UNIQUE_LOCK(M,L) std::unique_lock<std::remove_reference<decltype(M)>::type> L(M, std::chrono::milliseconds(MUTEX_TIMEOUT)); while (!L) { LOG(ERROR) << "Mutex timeout"; L.try_lock_for(std::chrono::milliseconds(MUTEX_TIMEOUT)); };
#define SHARED_LOCK(M,L) std::shared_lock<std::remove_reference<decltype(M)>::type> L(M, std::chrono::milliseconds(MUTEX_TIMEOUT)); while (!L) { LOG(ERROR) << "Mutex timeout"; L.try_lock_for(std::chrono::milliseconds(MUTEX_TIMEOUT)); };

#else
#define MUTEX std::mutex
#define RECURSIVE_MUTEX std::recursive_mutex
#define SHARED_MUTEX std::shared_mutex

#define UNIQUE_LOCK(M,L) std::unique_lock<std::remove_reference<decltype(M)>::type> L(M);
#define SHARED_LOCK(M,L) std::shared_lock<std::remove_reference<decltype(M)>::type> L(M);
#endif  // DEBUG_MUTEX

#define SHARED_LOCK_TYPE(M) std::shared_lock<M>

namespace ftl {
	extern ctpl::thread_pool pool;
}

#endif  // _FTL_THREADS_HPP_
