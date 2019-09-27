#ifndef _FTL_CUDA_WEIGHTING_HPP_
#define _FTL_CUDA_WEIGHTING_HPP_

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

}
}

#endif  // _FTL_CUDA_WEIGHTING_HPP_ 
