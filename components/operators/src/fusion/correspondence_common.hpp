#ifndef _FTL_OPERATORS_MVMLS_CORCOMMON_HPP_
#define _FTL_OPERATORS_MVMLS_CORCOMMON_HPP_

#ifndef PINF
#define PINF __int_as_float(0x7f800000)
#endif

/**
 * If the corresponding point has no depth or is off screen then it is bad.
 */
__device__ inline bool isBadCor(float depth, const float2 &pos, const ftl::rgbd::Camera &cam) {
	return
		depth <= cam.minDepth ||
		depth >= cam.maxDepth ||
		pos.x < 0.5f ||
		pos.y < 0.5f ||
		pos.x >= float(cam.width)-0.5f ||
		pos.y >= float(cam.height)-0.5f;
}

__device__ inline float square(float v) { return v*v; }

__device__ inline float length2(const float4 v) {
	return v.x*v.x + v.y*v.y + v.z*v.z;
}

template <int BLOCKS>
__device__ inline int2 block4x4() {
	return make_int2(
		(blockIdx.x*8 + (threadIdx.x%4) + 4*(threadIdx.x / 16)),
		blockIdx.y*4*BLOCKS + (threadIdx.x%16)/4 + 4*threadIdx.y
	);
}

#endif
