#ifndef _CUDATL_HALFWARP_HPP_
#define _CUDATL_HALFWARP_HPP_

#include <cuda_runtime.h>

namespace cudatl {

static constexpr int HALF_WARP_SIZE = 16;
static constexpr unsigned int HALF_MASK1 = 0xFFFF0000;
static constexpr unsigned int HALF_MASK2 = 0x0000FFFF;

template <typename T>
__device__ inline T halfWarpMin(T e) {
	for (int i = WARP_SIZE/4; i > 0; i /= 2) {
		const T other = __shfl_xor_sync(FULL_MASK, e, i, WARP_SIZE);
		e = min(e, other);
	}
	return e;
}

template <typename T>
__device__ inline T halfWarpMax(T e) {
	for (int i = WARP_SIZE/4; i > 0; i /= 2) {
		const T other = __shfl_xor_sync(FULL_MASK, e, i, WARP_SIZE);
		e = max(e, other);
	}
	return e;
}

template <typename T>
__device__ inline T halfWarpSum(T e) {
	for (int i = WARP_SIZE/4; i > 0; i /= 2) {
		const T other = __shfl_xor_sync(FULL_MASK, e, i, WARP_SIZE);
		e += other;
	}
	return e;
}

}

#endif
