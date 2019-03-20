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
 
#include <opencv2/core/cuda/common.hpp>

using namespace cv::cuda;
using namespace cv;

#define BLOCK_W 60
#define RADIUS 7
#define RADIUS2 2
#define ROWSperTHREAD 1

#define XHI(P1,P2) ((P1 <= P2) ? 0 : 1)

namespace ftl {
namespace gpu {

// --- SUPPORT -----------------------------------------------------------------

/*
 * Sparse 16x16 census (so 8x8) creating a 64bit mask
 * (14) & (15), based upon (9)
 */
__device__ uint64_t sparse_census(cudaTextureObject_t tex, int u, int v) {
	uint64_t r = 0;

	unsigned char t = tex2D<unsigned char>(tex, u,v);

	for (int m=-7; m<=7; m+=2) {
		//auto start_ix = (v + m)*w + u;
		for (int n=-7; n<=7; n+=2) {
			r <<= 1;
			r |= XHI(t, tex2D<unsigned char>(tex, u+n, v+m));
		}
	}

	return r;
}

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

/*
 * Calculate census mask for left and right images together.
 */
__global__ void census_kernel(cudaTextureObject_t l, cudaTextureObject_t r,
		int w, int h, uint64_t *censusL, uint64_t *censusR,
		size_t pL, size_t pR) {	
	
	int u = (blockIdx.x * BLOCK_W + threadIdx.x + RADIUS);
	int v_start = blockIdx.y * ROWSperTHREAD + RADIUS;
	int v_end = v_start + ROWSperTHREAD;
	
	if (v_end+RADIUS >= h) v_end = h-RADIUS;
	if (u+RADIUS >= w) return;
	
	for (int v=v_start; v<v_end; v++) {
		//int ix = (u + v*pL);
		uint64_t cenL = sparse_census(l, u, v);
		uint64_t cenR = sparse_census(r, u, v);
		
		censusL[(u + v*pL)] = cenL;
		censusR[(u + v*pR)] = cenR;
	}
}

__forceinline__ __device__ unsigned long long int int2_as_longlong (uint2 a)
{
    unsigned long long int res;
    asm ("mov.b64 %0, {%1,%2};" : "=l"(res) : "r"(a.x), "r"(a.y));
    return res;
}

/*
 * Generate left and right disparity images from census data. (19)
 */
__global__ void disp_kernel(float *disp_l, float *disp_r, size_t width, size_t height, cudaTextureObject_t censusL, cudaTextureObject_t censusR, size_t ds) {	
	//extern __shared__ uint64_t cache[];

	const int gamma = 5;
	
	size_t u = (blockIdx.x * BLOCK_W) + threadIdx.x + RADIUS2;
	size_t v_start = (blockIdx.y * ROWSperTHREAD) + RADIUS2;
	size_t v_end = v_start + ROWSperTHREAD;

	// Prepare the cache load
	//const int cache_thread_width = (BLOCK_W+ds / BLOCK_W + RADIUS2*2 + 1)*2;
	//uint64_t *cache_ptr = cache + (threadIdx.x * cache_thread_width);
	
	if (v_end >= height) v_end = height;
	//if (u >= width-ds) return;
	
	for (size_t v=v_start; v<v_end; v++) {
		/*const int cache_start = v*width*2 + cache_thread_width*blockIdx.x;
		for (int i=0; i<cache_thread_width; i+=2) {
			cache_ptr[i] = census[cache_start+i];
			cache_ptr[i+1] = census[cache_start+i+1];
		}
		
		__syncthreads();*/
		
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
		
		// TODO Use prediction textures to narrow range
		for (size_t d=0; d<ds; d++) {
			uint16_t hamming1 = 0;
			uint16_t hamming2 = 0;
			
			//if (u+2+ds >= width) break;
		
			for (int m=-2; m<=2; m++) {
				const auto v_ = (v + m);
				for (int n=-2; n<=2; n++) {
					const auto u_ = u + n;

				
					

					auto l2 = int2_as_longlong(tex2D<uint2>(censusL,u_,v_));
					auto l1 = int2_as_longlong(tex2D<uint2>(censusR,u_,v_));
					
					auto r1 = int2_as_longlong(tex2D<uint2>(censusL, u_+d, v_));
					auto r2 = int2_as_longlong(tex2D<uint2>(censusR, u_-d, v_));
					
					hamming1 += __popcll(r1^l1);
					hamming2 += __popcll(r2^l2);
				}
			}
			
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
		float d1 = fit_parabola(dix1, min_disp1, min_before1, min_after1);
		float d2 = fit_parabola(dix2, min_disp2, min_before2, min_after2);
	
		// Confidence filter (25)
		// TODO choice of gamma to depend on disparity variance
		// Variance with next option, variance with neighbours, variance with past value
		disp_l[v*width+u] = ((min_disp2b - min_disp2) >= gamma) ? d2 : INFINITY;
		disp_r[v*width+u] = ((min_disp1b - min_disp1) >= gamma) ? d1 : INFINITY;

		// TODO If disparity is 0.0f, perhaps
		// Use previous value unless it conflicts with present
		// Use neighbour values if texture matches 
	}
}

__global__ void consistency_kernel(float *d_sub_l, float *d_sub_r, PtrStepSz<float> disp) {
	size_t w = disp.cols;
	size_t h = disp.rows;
	//Mat result = Mat::zeros(Size(w,h), CV_32FC1);
	
	size_t u = (blockIdx.x * BLOCK_W) + threadIdx.x + RADIUS;
	size_t v_start = (blockIdx.y * ROWSperTHREAD) + RADIUS;
	size_t v_end = v_start + ROWSperTHREAD;
	
	if (v_end >= disp.rows) v_end = disp.rows;
	if (u >= w) return;
	
	for (size_t v=v_start; v<v_end; v++) {
	
		int a = (int)(d_sub_l[v*w+u]);
		if ((int)u-a < 0) continue;
		
		auto b = d_sub_r[v*w+u-a];
		
		if (abs(a-b) <= 1.0) disp(v,u) = abs((a+b)/2); // was 1.0
		else disp(v,u) = INFINITY;
	//}
	}

}

void rtcensus_call(const PtrStepSzb &l, const PtrStepSzb &r, const PtrStepSz<float> &disp, size_t num_disp, const int &stream) {
	dim3 grid(1,1,1);
    dim3 threads(BLOCK_W, 1, 1);

	grid.x = cv::cuda::device::divUp(l.cols - 2 * RADIUS, BLOCK_W);
	grid.y = cv::cuda::device::divUp(l.rows - 2 * RADIUS, ROWSperTHREAD);
	
	// TODO, reduce allocations
	uint64_t *censusL;
	uint64_t *censusR;
	float *disp_l;
	float *disp_r;
	size_t pitchL;
	size_t pitchR;
	cudaSafeCall( cudaMallocPitch(&censusL, &pitchL, l.cols*sizeof(uint64_t), l.rows) );
	cudaSafeCall( cudaMallocPitch(&censusR, &pitchR, r.cols*sizeof(uint64_t), r.rows) );
	
	//cudaMemset(census, 0, sizeof(uint64_t)*l.cols*l.rows*2);
	cudaMalloc(&disp_l, sizeof(float)*l.cols*l.rows);
	cudaMalloc(&disp_r, sizeof(float)*l.cols*l.rows);
	
	// Make textures
	cudaResourceDesc resDescL;
	memset(&resDescL, 0, sizeof(resDescL));
	resDescL.resType = cudaResourceTypePitch2D;
	resDescL.res.pitch2D.devPtr = l.data;
	resDescL.res.pitch2D.pitchInBytes = l.step;
	resDescL.res.pitch2D.desc = cudaCreateChannelDesc<unsigned char>();
	resDescL.res.pitch2D.width = l.cols;
	resDescL.res.pitch2D.height = l.rows;
	
	cudaResourceDesc resDescR;
	memset(&resDescR, 0, sizeof(resDescR));
	resDescR.resType = cudaResourceTypePitch2D;
	resDescR.res.pitch2D.devPtr = r.data;
	resDescR.res.pitch2D.pitchInBytes = r.step;
	resDescR.res.pitch2D.desc = cudaCreateChannelDesc<unsigned char>();
	resDescR.res.pitch2D.width = r.cols;
	resDescR.res.pitch2D.height = r.rows;
	
	cudaTextureDesc texDesc;
	memset(&texDesc, 0, sizeof(texDesc));
	texDesc.readMode = cudaReadModeElementType;
  
	cudaTextureObject_t texLeft = 0;
	cudaCreateTextureObject(&texLeft, &resDescL, &texDesc, NULL);
	cudaTextureObject_t texRight = 0;
	cudaCreateTextureObject(&texRight, &resDescR, &texDesc, NULL);

	//size_t smem_size = (2 * l.cols * l.rows) * sizeof(uint64_t);
	
	census_kernel<<<grid, threads>>>(texLeft, texRight, l.cols, l.rows, censusL, censusR, pitchL/sizeof(uint64_t), pitchR/sizeof(uint64_t));
	cudaSafeCall( cudaGetLastError() );
	
	//cudaSafeCall( cudaDeviceSynchronize() );
	
	// Make textures
	cudaResourceDesc censusLDesc;
	memset(&censusLDesc, 0, sizeof(censusLDesc));
	censusLDesc.resType = cudaResourceTypePitch2D;
	censusLDesc.res.pitch2D.devPtr = censusL;
	censusLDesc.res.pitch2D.pitchInBytes = pitchL;
	censusLDesc.res.pitch2D.desc = cudaCreateChannelDesc<uint2>();
	//censusLDesc.res.pitch2D.desc.filterMode = cudaFilterModePoint;
	censusLDesc.res.pitch2D.width = l.cols;
	censusLDesc.res.pitch2D.height = l.rows;
	
	cudaResourceDesc censusRDesc;
	memset(&censusRDesc, 0, sizeof(censusRDesc));
	censusRDesc.resType = cudaResourceTypePitch2D;
	censusRDesc.res.pitch2D.devPtr = censusR;
	censusRDesc.res.pitch2D.pitchInBytes = pitchR;
	censusRDesc.res.pitch2D.desc = cudaCreateChannelDesc<uint2>();
	//censusRDesc.res.pitch2D.desc.filterMode = cudaFilterModePoint;
	censusRDesc.res.pitch2D.width = r.cols;
	censusRDesc.res.pitch2D.height = r.rows;
  
	cudaTextureObject_t censusTexLeft = 0;
	cudaSafeCall( cudaCreateTextureObject(&censusTexLeft, &censusLDesc, &texDesc, NULL) );
	cudaTextureObject_t censusTexRight = 0;
	cudaSafeCall( cudaCreateTextureObject(&censusTexRight, &censusRDesc, &texDesc, NULL) );
	
	grid.x = cv::cuda::device::divUp(l.cols - 2 * RADIUS2, BLOCK_W);
	grid.y = cv::cuda::device::divUp(l.rows - 2 * RADIUS2, ROWSperTHREAD);
	
	//grid.x = cv::cuda::device::divUp(l.cols - 2 * RADIUS - num_disp, BLOCK_W) - 1;
	disp_kernel<<<grid, threads>>>(disp_l, disp_r, l.cols, l.rows, censusTexLeft, censusTexRight, num_disp);
	cudaSafeCall( cudaGetLastError() );
	
	consistency_kernel<<<grid, threads>>>(disp_l, disp_r, disp);
	cudaSafeCall( cudaGetLastError() );
	
	//if (&stream == Stream::Null())
	cudaSafeCall( cudaDeviceSynchronize() );
		
	cudaSafeCall( cudaDestroyTextureObject (texLeft) );
	cudaSafeCall( cudaDestroyTextureObject (texRight) );
	cudaSafeCall( cudaDestroyTextureObject (censusTexLeft) );
	cudaSafeCall( cudaDestroyTextureObject (censusTexRight) );
	cudaFree(disp_r);
	cudaFree(disp_l);
	cudaFree(censusL);
	cudaFree(censusR);
}

};
};
