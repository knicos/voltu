#ifndef _FTL_CUDA_HALF_HPP_
#define _FTL_CUDA_HALF_HPP_

#include <cuda_fp16.h>

struct half4 {
	half2 a;
	half2 b;
};

static_assert(sizeof(half4) == 8, "Incorrect half4 size");

template <>
__host__ inline cudaChannelFormatDesc cudaCreateChannelDesc<half4>() {
	return cudaCreateChannelDesc<short4>();
}

#ifdef __CUDACC__

// half4 functions
////////////////////////////////////////////////////////////////////////////////

inline __device__ half4 make_half4(half x, half y, half z, half w) {
	half4 h;
	h.a.x = x;
	h.a.y = y;
	h.b.x = z;
	h.b.y = w;
	return h;
}

inline __device__ half4 make_half4(half2 a, half2 b) {
	half4 h;
	h.a = a;
	h.b = b;
	return h;
}

inline __device__ half4 make_half4(float x, float y, float z, float w) {
	half4 h;
	h.a = __floats2half2_rn(x,y);
	h.b = __floats2half2_rn(z,w);
	return h;
}

inline __device__ half4 make_half4(const float4 &v) {
	half4 h;
	h.a = __floats2half2_rn(v.x,v.y);
	h.b = __floats2half2_rn(v.z,v.w);
	return h;
}

inline __device__ half4 make_half4(const float3 &v, float a) {
	half4 h;
	h.a = __floats2half2_rn(v.x,v.y);
	h.b = __floats2half2_rn(v.z,a);
	return h;
}

inline __device__ half4 make_half4(float a)
{
    return make_half4(a,a,a,a);
}

inline __device__ float4 make_float4(const half4 &v)
{
	union {
		struct {
			float2 a;
			float2 b;
		} x;
		float4 f; 
	};
	x.a = __half22float2(v.a);
	x.b = __half22float2(v.b);
	return f;
}

inline __device__ float3 make_float3(const half4 &v)
{
	union {
		struct {
			float2 a;
			float b;
		} x;
		float3 f; 
	};
	x.a = __half22float2(v.a);
	x.b = __half2float(v.b.x);
	return f;
}



// === Texture overloads =======================================================

template <>
__device__ inline half4 tex2D<half4>(cudaTextureObject_t o, float x, float y) {
	union U {
		__device__ inline U(const short4 &s) : t(s) {}
		short4 t;
		half4 h;
	};
	return U(tex2D<short4>(o, x, y)).h;
}

#endif


#endif