#ifndef _FTL_CUDA_WARP_HPP_
#define _FTL_CUDA_WARP_HPP_

#ifndef WARP_SIZE
#define WARP_SIZE 32
#endif

#define FULL_MASK 0xffffffff

namespace ftl {
namespace cuda {

__device__ inline float warpMin(float e) {
	for (int i = WARP_SIZE/2; i > 0; i /= 2) {
		const float other = __shfl_xor_sync(FULL_MASK, e, i, WARP_SIZE);
		e = min(e, other);
	}
	return e;
}

__device__ inline float warpMax(float e) {
	for (int i = WARP_SIZE/2; i > 0; i /= 2) {
		const float other = __shfl_xor_sync(FULL_MASK, e, i, WARP_SIZE);
		e = max(e, other);
	}
	return e;
}

__device__ inline float warpSum(float e) {
	for (int i = WARP_SIZE/2; i > 0; i /= 2) {
		const float other = __shfl_xor_sync(FULL_MASK, e, i, WARP_SIZE);
		e += other;
	}
	return e;
}

__device__ inline int warpSum(int e) {
	for (int i = WARP_SIZE/2; i > 0; i /= 2) {
		const float other = __shfl_xor_sync(FULL_MASK, e, i, WARP_SIZE);
		e += other;
	}
	return e;
}

// Half warp

__device__ inline float halfWarpMin(float e) {
	for (int i = WARP_SIZE/4; i > 0; i /= 2) {
		const float other = __shfl_xor_sync(FULL_MASK, e, i, WARP_SIZE);
		e = min(e, other);
	}
	return e;
}

__device__ inline float halfWarpMax(float e) {
	for (int i = WARP_SIZE/4; i > 0; i /= 2) {
		const float other = __shfl_xor_sync(FULL_MASK, e, i, WARP_SIZE);
		e = max(e, other);
	}
	return e;
}

__device__ inline float halfWarpSum(float e) {
	for (int i = WARP_SIZE/4; i > 0; i /= 2) {
		const float other = __shfl_xor_sync(FULL_MASK, e, i, WARP_SIZE);
		e += other;
	}
	return e;
}

__device__ inline int halfWarpSum(int e) {
	for (int i = WARP_SIZE/4; i > 0; i /= 2) {
		const float other = __shfl_xor_sync(FULL_MASK, e, i, WARP_SIZE);
		e += other;
	}
	return e;
}


}
}

#endif  // _FTL_CUDA_WARP_HPP_
