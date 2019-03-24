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

/* Convert vector uint2 (32bit x2) into a single uint64_t */
__forceinline__ __device__ uint64_t uint2asull (uint2 a) {
	uint64_t res;
	asm ("mov.b64 %0, {%1,%2};" : "=l"(res) : "r"(a.x), "r"(a.y));
	return res;
}

/*
 * Generate left and right disparity images from census data. (19)
 */
__global__ void disp_kernel(float *disp_l, float *disp_r,
		int pitchL, int pitchR,
		size_t width, size_t height,
		cudaTextureObject_t censusL, cudaTextureObject_t censusR,
		size_t ds) {	
	//extern __shared__ uint64_t cache[];

	const int gamma = 20;
	
	int u = (blockIdx.x * BLOCK_W) + threadIdx.x + RADIUS2;
	int v_start = (blockIdx.y * ROWSperTHREAD) + RADIUS2;
	int v_end = v_start + ROWSperTHREAD;
	int maxdisp = ds;
	
	// Local cache
	uint64_t l_cache_l1[5][5];
	uint64_t l_cache_l2[5][5];

	// Prepare the cache load
	//const int cache_thread_width = (BLOCK_W+ds / BLOCK_W + RADIUS2*2 + 1)*2;
	//uint64_t *cache_ptr = cache + (threadIdx.x * cache_thread_width);
	
	if (v_end >= height) v_end = height;
	if (u+maxdisp >= width) maxdisp = width-u;
	
	for (int v=v_start; v<v_end; v++) {
		/*const int cache_start = v*width*2 + cache_thread_width*blockIdx.x;
		for (int i=0; i<cache_thread_width; i+=2) {
			cache_ptr[i] = census[cache_start+i];
			cache_ptr[i+1] = census[cache_start+i+1];
		}
		
		__syncthreads();*/
		
		// Fill local cache for window 5x5
		// TODO Use shared memory?
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
		
		// TODO Use prediction textures to narrow range
		for (int d=0; d<maxdisp; d++) {
			uint16_t hamming1 = 0;
			uint16_t hamming2 = 0;
			
			//if (u+2+ds >= width) break;
		
			for (int m=-2; m<=2; m++) {
				const auto v_ = (v + m);
				for (int n=-2; n<=2; n++) {
					const auto u_ = u + n;
					
					auto l1 = l_cache_l1[m+2][n+2];
					auto l2 = l_cache_l2[m+2][n+2];
					
					// TODO Somehow might use shared memory
					auto r1 = uint2asull(tex2D<uint2>(censusL, u_+d, v_));
					auto r2 = uint2asull(tex2D<uint2>(censusR, u_-d, v_));
					
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
		disp_l[v*pitchL+u] = ((min_disp2b - min_disp2) >= gamma) ? d2 : NAN;
		disp_r[v*pitchR+u] = ((min_disp1b - min_disp1) >= gamma) ? d1 : NAN;

		// TODO If disparity is 0.0f, perhaps
		// Use previous value unless it conflicts with present
		// Use neighbour values if texture matches 
	}
}

/*
 * Check for consistency between the LR and RL disparity maps, only selecting
 * those that are similar. Otherwise it sets the disparity to NAN.
 */
__global__ void consistency_kernel(cudaTextureObject_t d_sub_l,
		cudaTextureObject_t d_sub_r, float *disp, int w, int h, int pitch) {

	// TODO This doesn't work at either edge (+-max_disparities)
	for (STRIDE_Y(v,h)) {
	for (STRIDE_X(u,w)) {
		float a = (int)tex2D<float>(d_sub_l, u, v);
		if (u-a < 0) {
			//disp[v*pitch+u] = a;
			continue;
		}
		
		auto b = tex2D<float>(d_sub_r, u-a, v);
		
		if (abs(a-b) <= 1.0) disp[v*pitch+u] = abs((a+b)/2);
		else disp[v*pitch+u] = NAN;
	}
	}

}



template <typename T>
__host__ __device__
inline T lerp(T v0, T v1, T t) {
    return fma(t, v1, fma(-t, v0, v0));
}

#define FILTER_WINDOW 21
#define FILTER_WINDOW_R	10
#define EDGE_SENSITIVITY 10.0f

__device__ float calculate_edge_disp(cudaTextureObject_t t, cudaTextureObject_t d, cudaTextureObject_t pT, cudaTextureObject_t pD, uchar4 pixel, int u, int v) {
	float est = 0.0;
	int nn = 0;
	//float pest = 0.0;
	//int pnn = 0;
	
	//cudaTextureObject_t nTex = (pT) ? pT : t;
	//cudaTextureObject_t nDisp = (pD) ? pD : d;

	for (int m=-FILTER_WINDOW_R; m<=FILTER_WINDOW_R; m++) {
		for (int n=-FILTER_WINDOW_R; n<=FILTER_WINDOW_R; n++) {
			uchar4 neigh = tex2D<uchar4>(t, u+n, v+m);
			float ndisp = tex2D<float>(d,u+n,v+m);
			
			//uchar4 pneigh = tex2D<uchar4>(nTex, u+n, v+m);
			//float pndisp = tex2D<float>(nDisp,u+n,v+m);

			//if (isnan(tex2D<float>(nDisp,u+n,v+m))) continue;
			//if (m == 0 && n == 0) continue;

			if (!isnan(ndisp) && (abs(neigh.z-pixel.z) <= EDGE_SENSITIVITY)) { // && (isnan(disp) || abs(ndisp-disp) < FILTER_DISP_THRESH)) {
				est += ndisp;
				nn++;
			}
			
			//if (!isnan(pndisp) && (abs(pneigh.z-pixel.z) <= EDGE_SENSITIVITY)) { // && (isnan(disp) || abs(ndisp-disp) < FILTER_DISP_THRESH)) {
			//	pest += pndisp;
			//	pnn++;
			//}
		}	
	}
	
	est = (nn > 0) ? est/nn : NAN;
	//pest = (pnn > 0) ? pest/pnn : NAN;
	
	return est;
}

__device__ float colour_error(uchar4 v1, uchar4 v2) {
	float dx = 0.05*abs(v1.x-v2.x);
	float dy = 0.1*abs(v1.y-v2.y);
	float dz = 0.85*abs(v1.z-v2.z);
	return dx + dz + dy;
}

// TODO Use HUE also and perhaps increase window?
// Or use more complex notion of texture?

/* Just crossed and currently on edge */
__device__ bool is_edge_left(uchar4 *line, int x, int n) {
	if (x < 1 || x >= n-1) return false;
	return (colour_error(line[x-1],line[x]) > EDGE_SENSITIVITY && colour_error(line[x],line[x+1]) <= EDGE_SENSITIVITY);
}

/* Just crossed but not on edge now */
__device__ bool is_edge_right(uchar4 *line, int x, int n) {
	if (x < 1 || x >= n-1) return false;
	return (colour_error(line[x-1],line[x]) <= EDGE_SENSITIVITY && colour_error(line[x],line[x+1]) > EDGE_SENSITIVITY);
}

__global__ void filter_kernel(cudaTextureObject_t t, cudaTextureObject_t d,
		cudaTextureObject_t prevD,
		cudaTextureObject_t prevT, PtrStepSz<float> f, int num_disp) {


	extern __shared__ uchar4 line[]; // One entire line of hsv image
	
	for (STRIDE_Y(v,f.rows)) {
		for (STRIDE_X(u,f.cols)) {
			line[u] = tex2D<uchar4>(t, u, v);
		}
		__syncthreads();
		
		for (STRIDE_X(u,f.cols)) {
			if (is_edge_right(line, u, f.cols)) {
				float edge_disp = calculate_edge_disp(t,d,prevT,prevD,line[u],u+2,v); // tex2D<float>(d, u, v);
				f(v,u) = edge_disp;
				continue;
				
				float est = 0.0f;
				int nn = 0;
				
				if (!isnan(edge_disp)) {
					est += edge_disp;
					nn++;
				}
				//f(v,u) = edge_disp;
				
				// TODO, find other edge first to get disparity
				// Use middle disparities to:
				//		estimate curve or linear (choose equation)
				//		or ignore as noise if impossible
				
				// TODO For edge disparity, use a window to:
				//		a) find a missing disparity
				//		b) make sure disparity has some consensus (above or below mostly)
				
				// TODO Use past values?
				// Another way to fill blanks and gain concensus
				
				// TODO Maintain a disparity stack to pop back to background?
				// Issue of background disparity not being detected.
				// Only if hsv also matches previous background
				
				// TODO Edge prediction (in vertical direction at least) could
				// help fill both edge and disparity gaps. Propagate disparity
				// along edges
				
				float last_disp = edge_disp;
				
				int i;
				for (i=1; u+i<f.cols; i++) {
					if (is_edge_right(line, u+i, f.cols)) {
						//float end_disp = calculate_edge_disp(t,d,prevT,prevD,line[u+i-1],u+i-3,v);
						//if (!isnan(end_disp)) last_disp = end_disp;
						break;
					}
					
					float di = tex2D<float>(d,u+i,v);
					if (!isnan(di)) {
						est += di;
						nn++;
					}
					//f(v,u+i) = edge_disp;
				}
				
				est = (nn > 0) ? est / nn : NAN;
				//for (int j=1; j<i; j++) {
				//	f(v,u+j) = est; //lerp(edge_disp, last_disp, (float)j / (float)(i-1));
				//}
			} else f(v,u) = NAN;
		}
	}
}

ftl::cuda::TextureObject<float> prevDisp;
ftl::cuda::TextureObject<uchar4> prevImage;

void rtcensus_call(const PtrStepSz<uchar4> &l, const PtrStepSz<uchar4> &r, const PtrStepSz<float> &disp, size_t num_disp, const int &stream) {
	dim3 grid(1,1,1);
    dim3 threads(BLOCK_W, 1, 1);

	grid.x = cv::cuda::device::divUp(l.cols - 2 * RADIUS, BLOCK_W);
	grid.y = cv::cuda::device::divUp(l.rows - 2 * RADIUS, ROWSperTHREAD);
	
	// TODO, reduce allocations
	uint64_t *censusL;
	uint64_t *censusR;
	size_t pitchL;
	size_t pitchR;

	float *disp_l;
	float *disp_r;
	size_t pitchDL;
	size_t pitchDR;

	float *disp_raw;
	size_t pitchD;
	
	cudaSafeCall( cudaMallocPitch(&censusL, &pitchL, l.cols*sizeof(uint64_t), l.rows) );
	cudaSafeCall( cudaMallocPitch(&censusR, &pitchR, r.cols*sizeof(uint64_t), r.rows) );
	
	//cudaMemset(census, 0, sizeof(uint64_t)*l.cols*l.rows*2);
	cudaSafeCall( cudaMallocPitch(&disp_l, &pitchDL, sizeof(float)*l.cols, l.rows) );
	cudaSafeCall( cudaMallocPitch(&disp_r, &pitchDR, sizeof(float)*l.cols, l.rows) );

	cudaSafeCall( cudaMallocPitch(&disp_raw, &pitchD, sizeof(float)*l.cols, l.rows) );
	
	cudaTextureDesc texDesc;
	memset(&texDesc, 0, sizeof(texDesc));
	texDesc.readMode = cudaReadModeElementType;
  
	ftl::cuda::TextureObject<uchar4> texLeft(l);
	ftl::cuda::TextureObject<uchar4> texRight(r);

	//size_t smem_size = (2 * l.cols * l.rows) * sizeof(uint64_t);
	
	// Calculate L and R census
	census_kernel<<<grid, threads>>>(texLeft.cudaTexture(), texRight.cudaTexture(), l.cols, l.rows, censusL, censusR, pitchL/sizeof(uint64_t), pitchR/sizeof(uint64_t));
	cudaSafeCall( cudaGetLastError() );
	
	//cudaSafeCall( cudaDeviceSynchronize() );
  
	ftl::cuda::TextureObject<uint2> censusTexLeft((uint2*)censusL, pitchL, l.cols, l.rows);
	ftl::cuda::TextureObject<uint2> censusTexRight((uint2*)censusR, pitchR, r.cols, r.rows);
	
	grid.x = cv::cuda::device::divUp(l.cols - 2 * RADIUS2, BLOCK_W);
	grid.y = cv::cuda::device::divUp(l.rows - 2 * RADIUS2, ROWSperTHREAD);
	
	// Calculate L and R disparities
	disp_kernel<<<grid, threads>>>(disp_l, disp_r, pitchDL/sizeof(float), pitchDR/sizeof(float), l.cols, l.rows, censusTexLeft.cudaTexture(), censusTexRight.cudaTexture(), num_disp);
	cudaSafeCall( cudaGetLastError() );

	ftl::cuda::TextureObject<float> dispTexLeft(disp_l, pitchDL, l.cols, l.rows);
	ftl::cuda::TextureObject<float> dispTexRight(disp_r, pitchDR, r.cols, r.rows);
	
	// Check consistency between L and R disparities.
	consistency_kernel<<<grid, threads>>>(dispTexLeft.cudaTexture(), dispTexRight.cudaTexture(), disp_raw, l.cols, l.rows, pitchD/sizeof(float));
	cudaSafeCall( cudaGetLastError() );

	ftl::cuda::TextureObject<float> dispTex(disp_raw, pitchD, r.cols, r.rows);

	grid.x = 4;
	grid.y = l.rows;
	threads.x = l.cols;
	size_t filter_smem = sizeof(uchar4) * l.cols;
	filter_kernel<<<grid, threads, filter_smem>>>(texLeft.cudaTexture(), dispTex.cudaTexture(), prevDisp.cudaTexture(), prevImage.cudaTexture(), disp, num_disp);
	cudaSafeCall( cudaGetLastError() );

	prevDisp.free();
	prevDisp = disp;
	prevImage.free();
	prevImage = texLeft;
	
	//if (&stream == Stream::Null())
	cudaSafeCall( cudaDeviceSynchronize() );
		
	//cudaSafeCall( cudaDestroyTextureObject (texLeft) );
	
	texRight.free();
	censusTexLeft.free();
	censusTexRight.free();
	dispTexLeft.free();
	dispTexRight.free();
	dispTex.free();
	
	cudaFree(disp_r);
	cudaFree(disp_l);
	cudaFree(censusL);
	cudaFree(censusR);
}

};
};
