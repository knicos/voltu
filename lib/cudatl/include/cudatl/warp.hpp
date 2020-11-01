#ifndef _CUDATL_WARP_HPP_
#define _CUDATL_WARP_HPP_

#include <cuda_runtime.h>

#define __cuda__ __host__ __device__

namespace cudatl {

static constexpr int WARP_SIZE = 32;
static constexpr unsigned int FULL_MASK = 0xFFFFFFFF;

template <typename T>
__device__ inline T warpMin(T e) {
	for (int i = WARP_SIZE/2; i > 0; i /= 2) {
		const T other = __shfl_xor_sync(FULL_MASK, e, i, WARP_SIZE);
		e = min(e, other);
	}
	return e;
}

template <typename T>
__device__ inline T warpMax(T e) {
	for (int i = WARP_SIZE/2; i > 0; i /= 2) {
		const T other = __shfl_xor_sync(FULL_MASK, e, i, WARP_SIZE);
		e = max(e, other);
	}
	return e;
}

template <typename T>
__device__ inline T warpSum(T e) {
	for (int i = WARP_SIZE/2; i > 0; i /= 2) {
		const T other = __shfl_xor_sync(FULL_MASK, e, i, WARP_SIZE);
		e += other;
	}
	return e;
}

/**
 * Find first histogram bucket that cumulatively exceeds a threshold, summing
 * all previous buckets. Note: s_Data must be 32 items.
 * TODO: This could be more efficient, perhaps with _shfl_XXX
 */
template <typename T>
inline __device__ int warpScan(volatile T *s_Data, int tix, T threshold) {
	const int thread = tix%32;
	for (uint offset = 1; offset < WARP_SIZE; offset <<= 1) {
		__syncwarp();
		const uint t = (thread >= offset) ? s_Data[thread] + s_Data[thread - offset] : s_Data[thread];
		__syncwarp();
		s_Data[thread] = t;
	}

	const uint t = __ballot_sync(FULL_MASK, s_Data[thread] > threshold);
	return __ffs(t);
}

}

#endif
