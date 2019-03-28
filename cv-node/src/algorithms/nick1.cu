/*
 * Author: Nicolas Pope
 * Based initially on rtcensus
 *
 */
 
#include <ftl/cuda_common.hpp>
#include <ftl/cuda_algorithms.hpp>

using namespace cv::cuda;
using namespace cv;


#define BLOCK_W 60
#define RADIUS 7
#define RADIUS2 2
#define ROWSperTHREAD 1

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

/*__global__ void filter_kernel(cudaTextureObject_t t, cudaTextureObject_t d,
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
}*/


__device__ float neighbour_factor(float a, cudaTextureObject_t p, int u, int v) {
	float f = 1.0f;
	
	for (int m=-1; m<=1; m++) {
		for (int n=-1; n<=1; n++) {
			float2 neighbour = tex2D<float2>(p, u+n, v+m);
			if (neighbour.x > 8.0f && abs(neighbour.y-a) < 1.0f) f += neighbour.x / 10.0f;
		}
	}
	
	return f;
}

/* Use Prewitt operator */
__global__ void edge_invar1_kernel(cudaTextureObject_t t, cudaTextureObject_t p, ftl::cuda::TextureObject<float2> o) {
	for (STRIDE_Y(v,o.height())) {
		for (STRIDE_X(u,o.width())) {
			float gx = ((tex2D<uchar4>(t, u-1, v-1).z - tex2D<uchar4>(t, u+1, v-1).z) +
						(tex2D<uchar4>(t, u-1, v).z - tex2D<uchar4>(t, u+1, v).z) +
						(tex2D<uchar4>(t, u-1, v+1).z - tex2D<uchar4>(t, u+1, v+1).z)) / 3;
			float gy = ((tex2D<uchar4>(t, u-1, v-1).z - tex2D<uchar4>(t, u-1, v+1).z) +
						(tex2D<uchar4>(t, u, v-1).z - tex2D<uchar4>(t, u, v+1).z) +
						(tex2D<uchar4>(t, u+1, v-1).z - tex2D<uchar4>(t, u+1, v+1).z)) / 3;
						
			float g = sqrt(gx*gx+gy*gy);
			float a = atan2(gy,gx);

			if (g > 1.0f) {
				float2 n = tex2D<float2>(p, u, v);
				float avg = (n.x > g && abs(n.y-a) < 0.2) ? (g+n.x) / 2.0f : g;
				o(u,v) = make_float2(avg,abs(a));
			} else {
				o(u,v) = make_float2(NAN,NAN);
			}
		}
	}
}

__device__ void edge_follow(float &sum, int &count, cudaTextureObject_t i1, int u, int v, int sign) {
	int u2 = u;
	int v2 = v;
	int n = 0;
	float sumchange = 0.0f;
	float2 pixel_i1 = tex2D<float2>(i1,u,v);

	for (int j=0; j<50; j++) {
		// Vertical edge = 0, so to follow it don't move in x
		int dx = ((pixel_i1.y >= 0.785 && pixel_i1.y <= 2.356) ) ? 0 : 1;
		int dy = (dx == 1) ? 0 : 1;

		// Check perpendicular to edge to find strongest gradient
		//if (tex2D<float2>(i1, u2+dy, v2+dx).x < pixel_i1.x && tex2D<float2>(i1, u2-dy, v2-dx).x < pixel_i1.x) {
			//o(u,v) = pixel_i1.y*81.0f;
		//} else {
		//	break;
		//}
		//continue;
		
		float2 next_pix;
		next_pix.x = NAN;
		next_pix.y = NAN;
		float diff = 10000.0f;
		int nu, nv;
		
		for (int i=-2; i<=2; i++) {
			float2 pix = tex2D<float2>(i1,u2+dx*i+dy*sign, v2+dy*i+dx*sign);
			if (isnan(pix.x)) continue;
			
			float d = abs(pix.x-pixel_i1.x)*abs(pix.y-pixel_i1.y);
			if (d < diff) {
				nu = u2+dx*i+dy*sign;
				nv = v2+dy*i+dx*sign;
				next_pix = pix;
				diff = d;
			}
		}
		
		if (!isnan(next_pix.x) && diff < 10.0f) {
			float change = abs(pixel_i1.y - next_pix.y);

			// Corner or edge change.
			//if (change > 0.785f) break;
			if (change > 1.0f) break;

			u2 = nu;
			v2 = nv;
			sumchange += change;
			pixel_i1 = next_pix;
			n++;
		} else {
			//o(u,v) = NAN;
			break;
		}
	}

	//if (n == 0) sum = 0.0f;
	sum = sumchange;
	count = n;
}

__global__ void edge_invar2_kernel(cudaTextureObject_t i1, ftl::cuda::TextureObject<float> o) {
	for (STRIDE_Y(v,o.height())) {
		for (STRIDE_X(u,o.width())) {
			float2 pixel_i1 = tex2D<float2>(i1,u,v);
			
			if (isnan(pixel_i1.x) || pixel_i1.x < 10.0f) {
				o(u,v) = NAN;
				continue;
			}

			int dx = ((pixel_i1.y >= 0.785 && pixel_i1.y <= 2.356) ) ? 1 : 0;
			int dy = (dx == 1) ? 0 : 1;

			// Check perpendicular to edge to find strongest gradient
			if (tex2D<float2>(i1, u+dy, v+dx).x < pixel_i1.x && tex2D<float2>(i1, u-dy, v-dx).x < pixel_i1.x) {
				//o(u,v) = pixel_i1.y*81.0f;
			} else {
				o(u,v) = NAN;
				continue;
			}

			float sum_a, sum_b;
			int count_a, count_b;
			edge_follow(sum_a, count_a, i1, u, v, 1);
			edge_follow(sum_b, count_b, i1, u, v, -1);
			

			// Output curvature of edge
			if (count_a+count_b > 10) {
				float curvature = ((sum_a+sum_b) / (float)(count_a+count_b));
				//o(u,v) = curvature * 300.0f + 50.0f;
				o(u,v) = (count_a+count_b) * 3.0f;
			} else {
				o(u,v) = NAN;
			}
			//o(u,v) = (sumchange / (float)(j-1))*100.0f;
			
			// Search in two directions for next edge pixel
			// Calculate curvature by monitoring gradient angle change
			// Calculate length by stopping when change exceeds threshold
		}	
	}
}

ftl::cuda::TextureObject<float2> prevEdge1;
ftl::cuda::TextureObject<float> prevDisp;
ftl::cuda::TextureObject<uchar4> prevImage;

namespace ftl {
namespace gpu {

void nick1_call(const PtrStepSz<uchar4> &l, const PtrStepSz<uchar4> &r, const PtrStepSz<float> &disp, size_t num_disp) {
	// Make all the required texture steps
	// TODO Could reduce re-allocations by caching these
	ftl::cuda::TextureObject<uchar4> texLeft(l);
	ftl::cuda::TextureObject<uchar4> texRight(r);
	ftl::cuda::TextureObject<float2> inv1(l.cols, l.rows);
	ftl::cuda::TextureObject<float> output(disp);
	
	dim3 grid(1,1,1);
    dim3 threads(BLOCK_W, 1, 1);
	grid.x = cv::cuda::device::divUp(l.cols - 2 * RADIUS2, BLOCK_W);
	grid.y = cv::cuda::device::divUp(l.rows - 2 * RADIUS2, ROWSperTHREAD);
	
	edge_invar1_kernel<<<grid,threads>>>(texLeft.cudaTexture(), prevEdge1.cudaTexture(), inv1);
	cudaSafeCall( cudaGetLastError() );
	
	edge_invar2_kernel<<<grid,threads>>>(inv1.cudaTexture(), output);
	cudaSafeCall( cudaGetLastError() );
	
	prevEdge1.free();
	prevEdge1 = inv1;
	
	//if (&stream == Stream::Null())
	cudaSafeCall( cudaDeviceSynchronize() );
	
	texLeft.free();
	texRight.free();
	//inv1.free();
	output.free();
}

}
}

