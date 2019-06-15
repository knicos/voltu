#ifndef _FTL_THREADS_HPP_
#define _FTL_THREADS_HPP_

#include <mutex>
#include <shared_mutex>

#if defined _DEBUG && DEBUG_MUTEX
#include <loguru.hpp>
#include <chrono>
#include <type_traits>

#define UNIQUE_LOCK(M,L) \
	auto start_##L = std::chrono::high_resolution_clock::now(); \
	std::unique_lock<std::remove_reference<decltype(M)>::type> L(M); \
	LOG(INFO) << "LOCK(" << #M "," #L << ") in " << std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_##L).count();

#define SHARED_LOCK(M,L) \
	auto start_##L = std::chrono::high_resolution_clock::now(); \
	std::shared_lock<std::remove_reference<decltype(M)>::type> L(M); \
	LOG(INFO) << "LOCK(" << #M "," #L << ") in " << std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_##L).count();
#else
#define UNIQUE_LOCK(M,L) std::unique_lock<std::remove_reference<decltype(M)>::type> L(M);
#define SHARED_LOCK(M,L) std::shared_lock<std::remove_reference<decltype(M)>::type> L(M);
#endif  // _DEBUG && DEBUG_MUTEX

namespace ftl {

}

#endif  // _FTL_THREADS_HPP_
