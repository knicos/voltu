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

#define BLOCK_W 128
#define RADIUS 7
#define RADIUS2 2
#define ROWSperTHREAD 20

#define XHI(P1,P2) ((P1 <= P2) ? 0 : 1)

namespace ftl {
namespace gpu {

__device__ uint64_t sparse_census(unsigned char *arr, size_t u, size_t v, size_t w) {
	uint64_t r = 0;

	unsigned char t = arr[v*w+u];

	for (int n=-7; n<=7; n+=2) {
	auto u_ = u + n;
	for (int m=-7; m<=7; m+=2) {
		auto v_ = v + m;
		r <<= 1;
		r |= XHI(t, arr[v_*w+u_]);
	}
	}

	return r;
}

__device__ float fit_parabola(size_t pi, uint16_t p, uint16_t pl, uint16_t pr) {
	float a = pr - pl;
	float b = 2 * (2 * p - pl - pr);
	return static_cast<float>(pi) + (a / b);
}

__global__ void census_kernel(PtrStepSzb l, PtrStepSzb r, uint64_t *census) {	
	//extern __shared__ uint64_t census[];
	
	size_t u = (blockIdx.x * BLOCK_W + threadIdx.x + RADIUS);
	size_t v_start = blockIdx.y * ROWSperTHREAD + RADIUS;
	size_t v_end = v_start + ROWSperTHREAD;
	
	if (v_end >= l.rows) v_end = l.rows;
	if (u >= l.cols) return;
	
	size_t width = l.cols;
	
	for (size_t v=v_start; v<v_end; v++) {
	//for (size_t u=7; u<width-7; u++) {
		size_t ix = (u + v*width) * 2;
		uint64_t cenL = sparse_census(l.data, u, v, l.step);
		uint64_t cenR = sparse_census(r.data, u, v, r.step);
		
		census[ix] = cenL;
		census[ix + 1] = cenR;
		
		//disp(v,u) = (float)cenL;
	//}
	}
	
	//__syncthreads();
	
	return;
}
	
__global__ void disp_kernel(float *disp_l, float *disp_r, size_t width, size_t height, uint64_t *census, size_t ds) {	
	//extern __shared__ uint64_t census[];
	
	size_t u = (blockIdx.x * BLOCK_W) + threadIdx.x + RADIUS2;
	size_t v_start = (blockIdx.y * ROWSperTHREAD) + RADIUS2;
	size_t v_end = v_start + ROWSperTHREAD;
	
	if (v_end >= height) v_end = height;
	//if (u >= width-ds) return;
	
	for (size_t v=v_start; v<v_end; v++) {
	//for (size_t u=7; u<width-7; u++) {
	//const size_t eu = (sign>0) ? w-2-ds : w-2;

	//for (size_t v=7; v<height-7; v++) {
	//for (size_t u=7; u<width-7; u++) {
		//const size_t ix = v*w*ds+u*ds;
		
		uint16_t last_ham[2] = {65535,65535};
		uint16_t min_disp[2] = {65535,65535};
		uint16_t min_before[2] = {0,0};
		uint16_t min_after[2] = {0,0};
		size_t dix[2] = {0,0};
		
		for (size_t d=0; d<ds; d++) {
			uint16_t hamming1 = 0;
			uint16_t hamming2 = 0;
			
			//if (u+2+ds >= width) break;
		
			for (int n=-2; n<=2; n++) {
				const auto u_ = u + n;

				for (int m=-2; m<=2; m++) {
					const auto v_ = (v + m)*width;

					// Correct for disp_R
					auto l1 = census[(u_+v_)*2+1];
					auto r1 = census[(v_+(u_+d))*2];
					
					// Correct for disp_L
					auto l2 = census[(u_+v_)*2];
					auto r2 = census[(v_+(u_-d))*2+1];
					
					hamming1 += __popcll(r1^l1);
					hamming2 += __popcll(r2^l2);
				}
			}
			
			if (hamming1 < min_disp[0]) {
				min_before[0] = last_ham[0];
				min_disp[0] = hamming1;
				dix[0] = d;
			}
			if (dix[0] == d) min_after[0] = hamming1;
			last_ham[0] = hamming1;
			
			if (hamming2 < min_disp[1]) {
				min_before[1] = last_ham[1];
				min_disp[1] = hamming2;
				dix[1] = d;
			}
			if (dix[1] == d) min_after[1] = hamming2;
			last_ham[1] = hamming2;
		
		}
		
		float d1 = (dix[0] == 0 || dix[0] == ds-1) ? (float)dix[0] : fit_parabola(dix[0], min_disp[0], min_before[0], min_after[0]);
		float d2 = (dix[1] == 0 || dix[1] == ds-1) ? (float)dix[1] : fit_parabola(dix[1], min_disp[1], min_before[1], min_after[1]);
	
		//if (abs(d1-d2) <= 1.0) disp(v,u) = abs((d1+d2)/2);
		//else disp(v,u) = 0.0f;
		
		//disp(v,u) = d1;
	
		disp_l[v*width+u] = d2;
		disp_r[v*width+u] = d1;
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
		
		if (abs(a-b) <= 1.0) disp(v,u) = abs((a+b)/2);
		else disp(v,u) = 0.0f;
	//}
	}

}

/*__global__ void test_kernel(const PtrStepSzb l, const PtrStepSzb r, PtrStepSz<float> disp)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x;
	int y = threadIdx.y + blockIdx.y * blockDim.y;
	if (x < l.cols && y < l.rows) {
		const unsigned char lv = l(y, x);
		const unsigned char rv = r(y, x);
		disp(y, x) = (float)lv - (float)rv; //make_uchar1(v.z, v.y, v.x);
	}
}*/

void rtcensus_call(const PtrStepSzb &l, const PtrStepSzb &r, const PtrStepSz<float> &disp, size_t num_disp, const int &stream) {
	dim3 grid(1,1,1);
    dim3 threads(BLOCK_W, 1, 1);

	grid.x = cv::cuda::device::divUp(l.cols - 2 * RADIUS, BLOCK_W);
	grid.y = cv::cuda::device::divUp(l.rows - 2 * RADIUS, ROWSperTHREAD);
	
	// TODO, reduce allocations
	uint64_t *census;
	float *disp_l;
	float *disp_r;
	cudaMalloc(&census, sizeof(uint64_t)*l.cols*l.rows*2);
	//cudaMemset(census, 0, sizeof(uint64_t)*l.cols*l.rows*2);
	cudaMalloc(&disp_l, sizeof(float)*l.cols*l.rows);
	cudaMalloc(&disp_r, sizeof(float)*l.cols*l.rows);
	
	//size_t smem_size = (2 * l.cols * l.rows) * sizeof(uint64_t);
	
	census_kernel<<<grid, threads>>>(l, r, census);
	cudaSafeCall( cudaGetLastError() );
	
	grid.x = cv::cuda::device::divUp(l.cols - 2 * RADIUS2, BLOCK_W);
	grid.y = cv::cuda::device::divUp(l.rows - 2 * RADIUS2, ROWSperTHREAD);
	
	//grid.x = cv::cuda::device::divUp(l.cols - 2 * RADIUS - num_disp, BLOCK_W) - 1;
	disp_kernel<<<grid, threads>>>(disp_l, disp_r, l.cols, l.rows, census, num_disp);
	cudaSafeCall( cudaGetLastError() );
	
	consistency_kernel<<<grid, threads>>>(disp_l, disp_r, disp);
	cudaSafeCall( cudaGetLastError() );
	
	cudaFree(disp_r);
	cudaFree(disp_l);
	cudaFree(census);
	
	//if (&stream == Stream::Null())
		cudaSafeCall( cudaDeviceSynchronize() );
}

};
};
