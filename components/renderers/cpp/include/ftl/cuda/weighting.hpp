#ifndef _FTL_CUDA_WEIGHTING_HPP_
#define _FTL_CUDA_WEIGHTING_HPP_

namespace ftl {
namespace cuda {

__device__ inline float weighting(float r, float h) {
	if (r >= h) return 0.0f;
	float rh = r / h;
	rh = 1.0f - rh*rh;
	return rh*rh*rh*rh;
}

/*
 * Guennebaud, G.; Gross, M. Algebraic point set surfaces. ACMTransactions on Graphics Vol. 26, No. 3, Article No. 23, 2007.
 * Used in: FusionMLS: Highly dynamic 3D reconstruction with consumer-grade RGB-D cameras
 *     r = distance between points
 *     h = smoothing parameter in meters (default 4cm)
 */
__device__ inline float spatialWeighting(const float3 &a, const float3 &b, float h) {
	const float r = length(a-b);
	if (r >= h) return 0.0f;
	float rh = r / h;
	rh = 1.0f - rh*rh;
	return rh*rh*rh*rh;
}

__device__ inline float colourDistance(uchar4 a, uchar4 b) {
	const float3 delta = make_float3((float)a.x - (float)b.x, (float)a.y - (float)b.y, (float)a.z - (float)b.z);
	return length(delta);
}

/*
 * Colour weighting as suggested in:
 * C. Kuster et al. Spatio-Temporal Geometry Fusion for Multiple Hybrid Cameras using Moving Least Squares Surfaces. 2014.
 * c = colour distance
 */
 __device__ inline float colourWeighting(uchar4 a, uchar4 b, float h) {
	const float3 delta = make_float3((float)a.x - (float)b.x, (float)a.y - (float)b.y, (float)a.z - (float)b.z);
	const float c = length(delta);
	if (c >= h) return 0.0f;
	float ch = c / h;
	ch = 1.0f - ch*ch;
	return ch*ch*ch*ch;
}

}
}

#endif  // _FTL_CUDA_WEIGHTING_HPP_ 
