#ifndef _FTL_MLS_CUDA_HPP_
#define _FTL_MLS_CUDA_HPP_

#include <ftl/cuda_util.hpp>
#include <ftl/cuda_common.hpp>
#include <ftl/cuda_matrix_util.hpp>
#include "splat_params.hpp"

__device__ inline float3 make_float3(const uchar4 &c) {
	return make_float3((float)c.x,(float)c.y,(float)c.z);
}

__device__ inline uchar4 make_uchar4(const float3 &c) {
	return make_uchar4(c.x,c.y,c.z,255);
}

namespace ftl {
namespace cuda {

/*
 * Guennebaud, G.; Gross, M. Algebraic point set surfaces. ACMTransactions on Graphics Vol. 26, No. 3, Article No. 23, 2007.
 * Used in: FusionMLS: Highly dynamic 3D reconstruction with consumer-grade RGB-D cameras
 *     r = distance between points
 *     h = smoothing parameter in meters (default 4cm)
 */
__device__ inline float spatialWeighting(float r, float h) {
	if (r >= h) return 0.0f;
	float rh = r / h;
	rh = 1.0f - rh*rh;
	return rh*rh*rh*rh;
}

__device__ float colourWeighting(float c);

struct fragment {
	float3 point;
	float3 normal;
	uchar4 colour;
};

__device__ inline float3 upsampled_point(cudaTextureObject_t pointset, const float2 &uv) {
	float3 R = make_float3(0.0f, 0.0f, 0.0f);
	const float3 P1 = make_float3(tex2D<float4>(pointset, int(uv.x), int(uv.y)));
	const float D1 = 1.0f - length(uv - make_float2(int(uv.x), int(uv.y)));
	R += D1 * P1;

	const float3 P2 = make_float3(tex2D<float4>(pointset, int(uv.x), int(uv.y+1.0f)));
	const float D2 = 1.0f - length(uv - make_float2(int(uv.x), int(uv.y+1.0f)));
	R += D2 * P2;

	const float3 P3 = make_float3(tex2D<float4>(pointset, int(uv.x+1.0f), int(uv.y)));
	const float D3 = 1.0f - length(uv - make_float2(int(uv.x+1.0f), int(uv.y)));
	R += D3 * P3;

	const float3 P4 = make_float3(tex2D<float4>(pointset, int(uv.x+1.0f), int(uv.y+1.0f)));
	const float D4 = 1.0f - length(uv - make_float2(int(uv.x+1.0f), int(uv.y+1.0f)));
	R += D4 * P4;

	// R is the centroid of surrounding points.
	R /= (D1+D2+D3+D4);

	// FIXME: Should not use centroid but instead sample the surface at this point
	// Use plane estimate at point to get "centroid" and then do the spatial weighted sample?
	return R;
}

__device__ inline fragment upsampled_point(cudaTextureObject_t pointset,
		cudaTextureObject_t normals, cudaTextureObject_t colours, const float2 &uv) {
	float3 R = make_float3(0.0f, 0.0f, 0.0f);
	float3 N = make_float3(0.0f, 0.0f, 0.0f);
	float3 C = make_float3(0.0f, 0.0f, 0.0f);

	// TODO:(Nick) Don't upsample points if distance is too great

	const float3 P1 = make_float3(tex2D<float4>(pointset, int(uv.x), int(uv.y)));
	const float D1 = 1.0f - length(uv - make_float2(int(uv.x), int(uv.y)));
	R += D1 * P1;
	N += D1 * make_float3(tex2D<float4>(normals, int(uv.x), int(uv.y)));
	C += D1 * make_float3(tex2D<uchar4>(colours, int(uv.x), int(uv.y)));

	const float3 P2 = make_float3(tex2D<float4>(pointset, int(uv.x), int(uv.y+1.0f)));
	const float D2 = 1.0f - length(uv - make_float2(int(uv.x), int(uv.y+1.0f)));
	R += D2 * P2;
	N += D2 * make_float3(tex2D<float4>(normals, int(uv.x), int(uv.y+1.0f)));
	C += D2 * make_float3(tex2D<uchar4>(colours, int(uv.x), int(uv.y+1.0f)));

	const float3 P3 = make_float3(tex2D<float4>(pointset, int(uv.x+1.0f), int(uv.y)));
	const float D3 = 1.0f - length(uv - make_float2(int(uv.x+1.0f), int(uv.y)));
	R += D3 * P3;
	N += D3 * make_float3(tex2D<float4>(normals, int(uv.x+1.0f), int(uv.y)));
	C += D3 * make_float3(tex2D<uchar4>(colours, int(uv.x+1.0f), int(uv.y)));

	const float3 P4 = make_float3(tex2D<float4>(pointset, int(uv.x+1.0f), int(uv.y+1.0f)));
	const float D4 = 1.0f - length(uv - make_float2(int(uv.x+1.0f), int(uv.y+1.0f)));
	R += D4 * P4;
	N += D4 * make_float3(tex2D<float4>(normals, int(uv.x+1.0f), int(uv.y+1.0f)));
	C += D4 * make_float3(tex2D<uchar4>(colours, int(uv.x+1.0f), int(uv.y+1.0f)));

	return {R / (D1+D2+D3+D4), N / (D1+D2+D3+D4), make_uchar4(C / (D1+D2+D3+D4))};
}

__device__ inline void render_depth(ftl::cuda::TextureObject<int> &depth, ftl::render::SplatParams &params, const float3 &worldPos) {
	const float3 camPos = params.m_viewMatrix * worldPos;
	const float d = camPos.z;
	if (d < params.camera.m_sensorDepthWorldMin) return;

	const float2 screenPosf = params.camera.cameraToKinectScreenFloat(camPos);
	const uint2 screenPos = make_uint2(make_int2(screenPosf));

	const unsigned int cx = screenPos.x;
	const unsigned int cy = screenPos.y;

	if (cx < depth.width() && cy < depth.height()) {
		atomicMin(&depth(cx,cy), d * 1000.0f);
	}
}

__device__ inline void render_fragment(
		ftl::cuda::TextureObject<int> &depth_in,
		ftl::cuda::TextureObject<float4> &normal_out,
		ftl::cuda::TextureObject<uchar4> &colour_out,
		ftl::render::SplatParams &params, const fragment &frag) {
	const float3 camPos = params.m_viewMatrix * frag.point;
	const float d = camPos.z;
	if (d < params.camera.m_sensorDepthWorldMin) return;

	const float2 screenPosf = params.camera.cameraToKinectScreenFloat(camPos);
	const uint2 screenPos = make_uint2(make_int2(screenPosf));

	const unsigned int cx = screenPos.x;
	const unsigned int cy = screenPos.y;

	if (cx < depth_in.width() && cy < depth_in.height()) {
		if (depth_in(cx,cy) == (int)(d * 1000.0f)) {
			colour_out(cx,cy) = frag.colour;
			normal_out(cx,cy) = make_float4(frag.normal, 0.0f);
		}
	}
}

/**
 * Estimate the point set surface location near to a given point.
 */
template <int R>
__device__ float3 screen_centroid(
		cudaTextureObject_t pointset,
		const float2 &suv,
		const int2 &uv,
		const ftl::render::SplatParams &params,
		float smoothing) {

	float3 pos = make_float3(0.0f, 0.0f, 0.0f);
	float weights = 0.0f;

	//#pragma unroll
	for (int v=-R; v<=R; ++v) {
		for (int u=-R; u<=R; ++u) {
			//if (uv.x+u >= 0 && uv.x+u < pointset.width() && uv.y+v >= 0 && uv.y+v < pointset.height()) {
				const float3 samplePoint = params.m_viewMatrix * make_float3(tex2D<float4>(pointset, uv.x+u, uv.y+v));
				const float weight = ftl::cuda::spatialWeighting(length(suv - params.camera.cameraToKinectScreenFloat(samplePoint)), smoothing);
				pos += weight*samplePoint;
				weights += weight;
			//}
		}
	}

	if (weights > 0.0f) pos = pos / weights;
	return pos;
}

/**
 * Estimate a point set surface point from an existing point in the set.
 */
template <int R>
__device__ float mls_point_surface(
		cudaTextureObject_t pointset,
		const int2 &uv,
		float3 &estPoint,
		float smoothing) {

	float3 pos = make_float3(0.0f, 0.0f, 0.0f);
	float weights = 0.0f;
	const float3 nearPoint = make_float3(tex2D<float4>(pointset, uv.x, uv.y));

	//#pragma unroll
	for (int v=-R; v<=R; ++v) {
		for (int u=-R; u<=R; ++u) {
			//if (uv.x+u >= 0 && uv.x+u < pointset.width() && uv.y+v >= 0 && uv.y+v < pointset.height()) {
				const float3 samplePoint = make_float3(tex2D<float4>(pointset, uv.x+u, uv.y+v));
				const float weight = ftl::cuda::spatialWeighting(length(nearPoint - samplePoint), smoothing);
				pos += weight*samplePoint;
				weights += weight;
			//}
		}
	}

	if (weights > 0.0f) estPoint = pos / weights;
	return weights;
};

/**
 * Estimate the point set surface location near to a given point.
 */
template <int R>
__device__ float mls_point_surface(
		cudaTextureObject_t pointset,
		const int2 &uv,
		const float3 &nearPoint,
		float3 &estPoint,
		float smoothing) {

	float3 pos = make_float3(0.0f, 0.0f, 0.0f);
	float weights = 0.0f;

	//#pragma unroll
	for (int v=-R; v<=R; ++v) {
		for (int u=-R; u<=R; ++u) {
			//if (uv.x+u >= 0 && uv.x+u < pointset.width() && uv.y+v >= 0 && uv.y+v < pointset.height()) {
				const float3 samplePoint = make_float3(tex2D<float4>(pointset, uv.x+u, uv.y+v));
				const float weight = ftl::cuda::spatialWeighting(length(nearPoint - samplePoint), smoothing);
				pos += weight*samplePoint;
				weights += weight;
			//}
		}
	}

	if (weights > 0.0f) estPoint = pos / weights;
	return weights;
}

/**
 * Calculate the point sample energy.
 */
template <int R>
__device__ float mls_point_energy(
		cudaTextureObject_t pointset,
		const int2 &uv,
		const float3 &nearPoint,
		float smoothing) {

	float weights = 0.0f;

	//#pragma unroll
	for (int v=-R; v<=R; ++v) {
		for (int u=-R; u<=R; ++u) {
			//if (uv.x+u >= 0 && uv.x+u < pointset.width() && uv.y+v >= 0 && uv.y+v < pointset.height()) {
				const float3 samplePoint = make_float3(tex2D<float4>(pointset, uv.x+u, uv.y+v));
				const float weight = ftl::cuda::spatialWeighting(length(nearPoint - samplePoint), smoothing);
				weights += weight;
			//}
		}
	}

	return weights;
}

/**
 * Calculate the point sample energy.
 */
template <int M>
__device__ float mls_point_energy(
		const float3 (&pointset)[M],
		const float3 &nearPoint,
		float smoothing) {

	float weights = 0.0f;

	//#pragma unroll
	for (int i=0; i<M; ++i) {
		const float3 samplePoint = pointset[i];
		const float weight = ftl::cuda::spatialWeighting(length(nearPoint - samplePoint), smoothing);
		weights += weight;
	}

	return weights;
}

/**
 * Estimate a point set surface location near an existing and return also
 * an estimate of the normal and colour of that point.
 */
template <int R>
__device__ float mls_point_surface(
		cudaTextureObject_t pointset,
		cudaTextureObject_t normalset,
		cudaTextureObject_t colourset,
		const int2 &uv,
		float3 &estPoint,
		float3 &estNormal,
		uchar4 &estColour,
		float smoothing) {

	float3 pos = make_float3(0.0f, 0.0f, 0.0f);
	float3 normal = make_float3(0.0f, 0.0f, 0.0f);
	float3 colour = make_float3(0.0f, 0.0f, 0.0f);
	float weights = 0.0f;
	const float3 nearPoint = make_float3(tex2D<float4>(pointset, uv.x, uv.y));

	//#pragma unroll
	for (int v=-R; v<=R; ++v) {
		for (int u=-R; u<=R; ++u) {
			//if (uv.x+u >= 0 && uv.x+u < pointset.width() && uv.y+v >= 0 && uv.y+v < pointset.height()) {
				const float3 samplePoint = make_float3(tex2D<float4>(pointset, uv.x+u, uv.y+v));
				const float weight = spatialWeighting(length(nearPoint - samplePoint), smoothing);

				if (weight > 0.0f) {
					pos += weight*samplePoint;
					weights += weight;

					normal += weight * make_float3(tex2D<float4>(normalset, uv.x+u, uv.y+v));
					const uchar4 c = tex2D<uchar4>(colourset, uv.x+u, uv.y+v);
					colour += weight * make_float3(c.x, c.y, c.z);
				}
			//}
		}
	}

	if (weights > 0.0f) {
		estPoint = pos / weights;
		estNormal = normal / weights;
		estColour = make_uchar4(colour.x / weights, colour.y / weights, colour.z / weights, 255);
	}
	return weights;
}

/**
 * Estimate a point set surface location near a given point and return also
 * an estimate of the normal and colour of that point.
 */
template <int R>
__device__ float mls_point_surface(
		cudaTextureObject_t pointset,
		cudaTextureObject_t normalset,
		cudaTextureObject_t colourset,
		const int2 &uv,
		const float3 &nearPoint,
		float3 &estPoint,
		float3 &estNormal,
		uchar4 &estColour,
		float smoothing) {

	float3 pos = make_float3(0.0f, 0.0f, 0.0f);
	float3 normal = make_float3(0.0f, 0.0f, 0.0f);
	float3 colour = make_float3(0.0f, 0.0f, 0.0f);
	float weights = 0.0f;

	//#pragma unroll
	for (int v=-R; v<=R; ++v) {
		for (int u=-R; u<=R; ++u) {
			//if (uv.x+u >= 0 && uv.x+u < pointset.width() && uv.y+v >= 0 && uv.y+v < pointset.height()) {
				const float3 samplePoint = make_float3(tex2D<float4>(pointset, uv.x+u, uv.y+v));
				const float weight = spatialWeighting(length(nearPoint - samplePoint), smoothing);

				if (weight > 0.0f) {
					pos += weight*samplePoint;
					weights += weight;

					normal += weight * make_float3(tex2D<float4>(normalset, uv.x+u, uv.y+v));
					const uchar4 c = tex2D<uchar4>(colourset, uv.x+u, uv.y+v);
					colour += weight * make_float3(c.x, c.y, c.z);
				}
			//}
		}
	}

	if (weights > 0.0f) {
		estPoint = pos / weights;
		estNormal = normal / weights;
		estColour = make_uchar4(colour.x / weights, colour.y / weights, colour.z / weights, 255);
	}
	return weights;
}

}
}

#endif  // _FTL_MLS_CUDA_HPP_
