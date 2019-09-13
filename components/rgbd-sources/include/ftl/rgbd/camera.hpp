#pragma once
#ifndef _FTL_RGBD_CAMERA_PARAMS_HPP_
#define _FTL_RGBD_CAMERA_PARAMS_HPP_

#include <vector_types.h>
#include <cuda_runtime.h>
#include <ftl/cuda_util.hpp>

namespace ftl{
namespace rgbd {

/**
 * All properties associated with cameras. This structure is designed to
 * operate on CPU and GPU.
 */
struct __align__(16) Camera {
	double fx;				// Focal length X
	double fy;				// Focal length Y (usually same as fx)
	double cx;				// Principle point Y
	double cy;				// Principle point Y
	unsigned int width;		// Pixel width
	unsigned int height;	// Pixel height
	double minDepth;		// Near clip in meters
	double maxDepth;		// Far clip in meters
	double baseline;		// For stereo pair
	double doffs;			// Disparity offset

	/**
	 * Convert camera coordinates into screen coordinates.
	 */
	template <typename T> __device__ T camToScreen(const float3 &pos) const;

	/**
	 * Convert screen plus depth into camera coordinates.
	 */
	__device__ float3 screenToCam(uint ux, uint uy, float depth) const; 
};

};
};

// ---- IMPLEMENTATIONS --------------------------------------------------------

template <> __device__
inline float2 ftl::rgbd::Camera::camToScreen<float2>(const float3 &pos) const {
	return make_float2(
			pos.x*fx/pos.z + cx,			
			pos.y*fy/pos.z + cy);
}

template <> __device__
inline int2 ftl::rgbd::Camera::camToScreen<int2>(const float3 &pos) const {
	float2 pImage = camToScreen<float2>(pos);
	return make_int2(pImage + make_float2(0.5f, 0.5f));
}

template <> __device__
inline uint2 ftl::rgbd::Camera::camToScreen<uint2>(const float3 &pos) const {
	int2 p = camToScreen<int2>(pos);
	return make_uint2(p.x, p.y);
}

__device__
inline float3 ftl::rgbd::Camera::screenToCam(uint ux, uint uy, float depth) const {
	const float x = ((float)ux-cx) / fx;
	const float y = ((float)uy-cy) / fy;
	return make_float3(depth*x, depth*y, depth);
}

#endif
