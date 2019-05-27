#pragma once

#ifndef _CUDA_UTIL_
#define _CUDA_UTIL_

#undef max
#undef min

#include <cuda_runtime.h>
#include <ftl/cuda_operators.hpp>

// Enable run time assertion checking in kernel code
#define cudaAssert(condition) if (!(condition)) { printf("ASSERT: %s %s\n", #condition, __FILE__); }
//#define cudaAssert(condition)

#if defined(__CUDA_ARCH__)
#define __CONDITIONAL_UNROLL__ #pragma unroll
#else
#define __CONDITIONAL_UNROLL__ 
#endif

#endif
