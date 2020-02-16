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
		pos.x >= cam.width-0.5f ||
		pos.y >= cam.height-0.5f;
}

__device__ inline float square(float v) { return v*v; }

#endif
