/*
 * Author: Nicolas Pope and Sebastian Hahta (2019)
 * Implementation of algorithm presented in article(s):
 *
 * [1] Humenberger, Engelke, Kubinger: A fast stereo matching algorithm suitable
 *     for embedded real-time systems
 * [2] Humenberger, Zinner, Kubinger: Performance Evaluation of Census-Based
 *     Stereo Matching Algorithm on Embedded and Multi-Core Hardware
 *
 * Equation numbering uses [1] unless otherwise stated
 *
 */

#include <ftl/cuda_common.hpp>

#define XHI(P1,P2) ((P1 <= P2) ? 0 : 1)

/*
 * Sparse 16x16 census (so 8x8) creating a 64bit mask
 * (14) & (15), based upon (9)
 */
__device__ uint64_t _sparse_census(cudaTextureObject_t tex, int u, int v) {
	uint64_t r = 0;

	unsigned char t = tex2D<uchar4>(tex, u,v).z;

	for (int m=-7; m<=7; m+=2) {
		//auto start_ix = (v + m)*w + u;
		for (int n=-7; n<=7; n+=2) {
			r <<= 1;
			r |= XHI(t, tex2D<uchar4>(tex, u+n, v+m).z);
		}
	}

	return r;
}

/*
 * Calculate census mask for left and right images together.
 */
__global__ void census_kernel(cudaTextureObject_t l, cudaTextureObject_t r,
		int w, int h, uint64_t *censusL, uint64_t *censusR,
		size_t pL, size_t pR) {	
	
	//if (v_end+RADIUS >= h) v_end = h-RADIUS;
	//if (u+RADIUS >= w) return;
	
	for (STRIDE_Y(v,h)) {
	for (STRIDE_X(u,w)) {
		//int ix = (u + v*pL);
		uint64_t cenL = _sparse_census(l, u, v);
		uint64_t cenR = _sparse_census(r, u, v);
		
		censusL[(u + v*pL)] = cenL;
		censusR[(u + v*pR)] = cenR;
	}
	}
}

namespace ftl {
namespace cuda {
	void sparse_census(const TextureObject<uchar4> &l, const TextureObject<uchar4> &r,
			TextureObject<uint2> &cl, TextureObject<uint2> &cr) {
		dim3 grid(1,1,1);
    	dim3 threads(128, 1, 1);
    	grid.x = cv::cuda::device::divUp(l.width(), 128);
		grid.y = cv::cuda::device::divUp(l.height(), 11);
		
		
		census_kernel<<<grid, threads>>>(
			l.cudaTexture(), r.cudaTexture(),
			l.width(), l.height(),
			(uint64_t*)cl.devicePtr(), (uint64_t*)cr.devicePtr(),
			cl.pitch()/sizeof(uint64_t),
			cr.pitch()/sizeof(uint64_t));
		cudaSafeCall( cudaGetLastError() );
	}
}
}


