/*
 * Author: Nicolas Pope and Sebastian Hahta (2019)
 * Implementation of algorithm presented in article(s):
 *
 * [1] Humenberger, Engelke, Kubinger: A fast stereo matching algorithm suitable
 *     for embedded real-time systems
 * [2] Humenberger, Zinner, Kubinger: Performance Evaluation of Census-Based
 *     Stereo Matching Algorithm on Embedded and Multi-Core Hardware
 * [3] Humenberger, Engelke, Kubinger: A Census-Based Stereo Vision Algorithm Using Modified Semi-Global Matching
 *     and Plane Fitting to Improve Matching Quality.
 *
 * Equation numbering uses [1] unless otherwise stated
 *
 */
 
#include <ftl/cuda_common.hpp>
#include "../cuda_algorithms.hpp"

using namespace cv::cuda;
using namespace cv;


#define BLOCK_W 60
#define RADIUS 7
#define RADIUS2 2
#define ROWSperTHREAD 1

namespace ftl {
namespace gpu {

// --- SUPPORT -----------------------------------------------------------------


/*
 * Parabolic interpolation between matched disparities either side.
 * Results in subpixel disparity. (20).
 */
__device__ float fit_parabola(size_t pi, uint16_t p, uint16_t pl, uint16_t pr) {
	float a = pr - pl;
	float b = 2 * (2 * p - pl - pr);
	return static_cast<float>(pi) + (a / b);
}

// --- KERNELS -----------------------------------------------------------------

/* Convert vector uint2 (32bit x2) into a single uint64_t */
__forceinline__ __device__ uint64_t uint2asull (uint2 a) {
	uint64_t res;
	asm ("mov.b64 %0, {%1,%2};" : "=l"(res) : "r"(a.x), "r"(a.y));
	return res;
}

/*
 * Generate left and right disparity images from census data. (18)(19)(25)
 */
__global__ void disp_kernel(float *disp_l, float *disp_r,
		int pitchL, int pitchR,
		size_t width, size_t height,
		cudaTextureObject_t censusL, cudaTextureObject_t censusR,
		size_t ds) {	
	//extern __shared__ uint64_t cache[];

	const int gamma = 35;
	
	int u = (blockIdx.x * BLOCK_W) + threadIdx.x + RADIUS2;
	int v_start = (blockIdx.y * ROWSperTHREAD) + RADIUS2;
	int v_end = v_start + ROWSperTHREAD;
	int maxdisp = ds;
	
	// Local cache
	uint64_t l_cache_l1[5][5];
	uint64_t l_cache_l2[5][5];
	
	if (v_end >= height) v_end = height;
	if (u+maxdisp >= width) maxdisp = width-u;
	
	for (int v=v_start; v<v_end; v++) {
		for (int m=-2; m<=2; m++) {
			for (int n=-2; n<=2; n++) {
				l_cache_l2[m+2][n+2] = uint2asull(tex2D<uint2>(censusL,u+n,v+m));
				l_cache_l1[m+2][n+2] = uint2asull(tex2D<uint2>(censusR,u+n,v+m));
			}
		}
		
		uint16_t last_ham1 = 65535;
		uint16_t last_ham2 = 65535;
		uint16_t min_disp1 = 65535;
		uint16_t min_disp2 = 65535;
		uint16_t min_disp1b = 65535;
		uint16_t min_disp2b = 65535;
		uint16_t min_before1 = 0;
		uint16_t min_before2 = 0;
		uint16_t min_after1 = 0;
		uint16_t min_after2 = 0;
		int dix1 = 0;
		int dix2 = 0;
		
		// (19)
		for (int d=0; d<maxdisp; d++) {
			uint16_t hamming1 = 0;
			uint16_t hamming2 = 0;
			
			//if (u+2+ds >= width) break;
		
			for (int m=-2; m<=2; m++) {
				const auto v_ = (v + m);
				for (int n=-2; n<=2; n++) {
					const auto u_ = u + n;
					
					// (18)
					auto l1 = l_cache_l1[m+2][n+2];
					auto l2 = l_cache_l2[m+2][n+2];
					auto r1 = uint2asull(tex2D<uint2>(censusL, u_+d, v_));
					auto r2 = uint2asull(tex2D<uint2>(censusR, u_-d, v_));
					hamming1 += __popcll(r1^l1);
					hamming2 += __popcll(r2^l2);
				}
			}
			
			// Find the two minimum costs
			if (hamming1 < min_disp1) {
				min_before1 = last_ham1;
				min_disp1 = hamming1;
				dix1 = d;
			} else if (hamming1 < min_disp1b) {
				min_disp1b = hamming1;
			}
			if (dix1 == d) min_after1 = hamming1;
			last_ham1 = hamming1;
			
			if (hamming2 < min_disp2) {
				min_before2 = last_ham2;
				min_disp2 = hamming2;
				dix2 = d;
			} else if (hamming2 < min_disp2b) {
				min_disp2b = hamming2;
			}
			if (dix2 == d) min_after2 = hamming2;
			last_ham2 = hamming2;
		
		}
		
		//float d1 = (dix1 == 0 || dix1 == ds-1) ? (float)dix1 : fit_parabola(dix1, min_disp1, min_before1, min_after1);
		//float d2 = (dix2 == 0 || dix2 == ds-1) ? (float)dix2 : fit_parabola(dix2, min_disp2, min_before2, min_after2);
	
		// TODO Allow for discontinuities with threshold
		// Subpixel disparity (20)
		float d1 = fit_parabola(dix1, min_disp1, min_before1, min_after1);
		float d2 = fit_parabola(dix2, min_disp2, min_before2, min_after2);
	
		// Confidence filter based on (25)
		disp_l[v*pitchL+u] = ((min_disp2b - min_disp2) >= gamma) ? d2 : NAN;
		disp_r[v*pitchR+u] = ((min_disp1b - min_disp1) >= gamma) ? d1 : NAN;
	}
}

void rtcensus_call(const PtrStepSz<uchar4> &l, const PtrStepSz<uchar4> &r, const PtrStepSz<float> &disp, size_t num_disp, const int &stream) {
	// Make all the required texture steps
	// TODO Could reduce re-allocations by caching these
	ftl::cuda::TextureObject<uchar4> texLeft(l);
	ftl::cuda::TextureObject<uchar4> texRight(r);
	ftl::cuda::TextureObject<uint2> censusTexLeft(l.cols, l.rows);
	ftl::cuda::TextureObject<uint2> censusTexRight(r.cols, r.rows);
	ftl::cuda::TextureObject<float> dispTexLeft(l.cols, l.rows);
	ftl::cuda::TextureObject<float> dispTexRight(r.cols, r.rows);
	ftl::cuda::TextureObject<float> dispTex(r.cols, r.rows);
	ftl::cuda::TextureObject<float> output(disp);
	
	// Calculate the census for left and right (14)(15)(16)
	ftl::cuda::sparse_census(texLeft, texRight, censusTexLeft, censusTexRight);

	dim3 grid(1,1,1);
    dim3 threads(BLOCK_W, 1, 1);
	grid.x = cv::cuda::device::divUp(l.cols - 2 * RADIUS2, BLOCK_W);
	grid.y = cv::cuda::device::divUp(l.rows - 2 * RADIUS2, ROWSperTHREAD);
	
	// Calculate L and R disparities (18)(19)(20)(21)(22)(25)
	disp_kernel<<<grid, threads>>>(
		dispTexLeft.devicePtr(), dispTexRight.devicePtr(),
		dispTexLeft.pitch()/sizeof(float), dispTexRight.pitch()/sizeof(float),
		l.cols, l.rows,
		censusTexLeft.cudaTexture(), censusTexRight.cudaTexture(),
		num_disp);
	cudaSafeCall( cudaGetLastError() );
	
	// Check consistency between L and R disparities. (23)(24)
	consistency(dispTexLeft, dispTexRight, dispTex);

	// TM in (7) of paper [3]. Eq (26) in [1] is wrong.
	texture_filter(texLeft, dispTex, output, num_disp, 10.0);

	cudaSafeCall( cudaDeviceSynchronize() );
	
	texLeft.free();
	texRight.free();
	censusTexLeft.free();
	censusTexRight.free();
	dispTexLeft.free();
	dispTexRight.free();
	dispTex.free();
	output.free();
}

};
};
