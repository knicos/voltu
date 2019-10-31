#ifndef _FTL_CUDA_MAKERS_HPP_
#define _FTL_CUDA_MAKERS_HPP_

#include <ftl/cuda_common.hpp>

__device__ inline float4 make_float4(const uchar4 &c) {
    return make_float4(c.x,c.y,c.z,c.w);
}

__device__ inline float4 make_float4(const float4 &v) {
	return v;
}

template <typename T>
__device__ inline T make();

template <>
__device__ inline uchar4 make() {
	return make_uchar4(0,0,0,0);
}

template <>
__device__ inline float4 make() {
	return make_float4(0.0f,0.0f,0.0f,0.0f);
}

template <>
__device__ inline float make() {
	return 0.0f;
}

template <typename T>
__device__ inline T make(const float4 &);

template <>
__device__ inline uchar4 make(const float4 &v) {
	return make_uchar4((int)v.x, (int)v.y, (int)v.z, (int)v.w);
}

template <>
__device__ inline float4 make(const float4 &v) {
	return v;
}

template <>
__device__ inline float make(const float4 &v) {
	return v.x;
}

template <typename T>
__device__ inline T make(const uchar4 &v);

template <>
__device__ inline float4 make(const uchar4 &v) {
	return make_float4((float)v.x, (float)v.y, (float)v.z, (float)v.w);
}

template <typename T>
__device__ inline T make(float v);

template <>
__device__ inline float make(float v) {
	return v;
}

#endif  // _FTL_CUDA_MAKERS_HPP_
