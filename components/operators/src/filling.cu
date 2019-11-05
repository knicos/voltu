#include "filling_cuda.hpp"

#define WARP_SIZE 32
#define MAX_EDGES 128

using ftl::cuda::TextureObject;

__device__ inline bool isValid(const ftl::rgbd::Camera &camera, float d) {
	return d > camera.minDepth && d < camera.maxDepth; 
}

__global__ void scan_field_fill_kernel(
		TextureObject<float> depth_in,        // Virtual depth map
		TextureObject<float> depth_out,   // Accumulated output
		TextureObject<float> smoothing,
		float thresh,
		ftl::rgbd::Camera camera) {

	__shared__ int counter;
	__shared__ int offsets[MAX_EDGES];
		
	//const int x = blockIdx.x*blockDim.x + threadIdx.x;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (y >= depth_in.height()) return;
	if (threadIdx.x == 0) counter = 0;
	__syncthreads();

	for (STRIDE_X(x,depth_in.width()-1)) {

		// Use entire block to find depth edges and make a list of them
		float d0 = depth_in.tex2D(x,y);
		float d1 = depth_in.tex2D(x+1,y);

		if (isValid(camera, d0) != isValid(camera, d1)) {
			int ix = atomicAdd(&counter, 1);
			if (ix >= MAX_EDGES) break;
			offsets[ix] = x;
		}
	}

	__syncthreads();

	// For each identified pixel, a thread takes on a filling job
	for (int i=threadIdx.x; i<counter; i += blockDim.x) {
		// Get smoothing gradient to decide action
		int x = offsets[i];

		float s0 = smoothing.tex2D(x,y);
		float d0 = depth_in.tex2D(x,y);
		float d1 = depth_in.tex2D(x+1,y);

		// More than 8 pixels from edge
		if (s0 < thresh) continue;
		//if (s0 < 2.0f / 100.0f) continue;

		/*bool increase = false;
		for (int u=1; u<20; ++u) {
			float s1 = smoothing.tex2D(x+u,y);
			if (s1 != s0) {
				increase = s1 > s0;
				break;
			}
		}*/

		// TODO: Smoothing channel has discrete steps meaning gradient can't be determined? CHECK
		// The above would be why some slices are not filled

		//bool fill_right = (isValid(camera, d0) && !increase);
		//bool clear_right = (isValid(camera, d1) && !increase);
		//bool fill_left = (isValid(camera, d1) && increase);
		//bool clear_left = (isValid(camera, d0) && increase);
		//bool clear = clear_left || clear_right;

		bool fill_right = isValid(camera, d0);

		//if (s1 == s0 || max(s1,s0) > 0.1f) continue;

		// Set up fill value and direction
		//float value = ((clear_left || clear_right) && s0 < thresh) ? 1000.0f : (isValid(camera, d0) ? d0 : d1);
		//int dir = (fill_right || clear_right) ? 1 : -1;

		float value = (isValid(camera, d0)) ? d0 : d1;
		int dir = (fill_right) ? 1 : -1;

		// TODO: The minimum needs to be search for and not crossed into, the filling
		// must stop before the minimum. Requires lookahead.

		/*int end_x = 0;
		for (int u=1; u<1000; ++u) {
			float s = smoothing(x+u*dir, y);
			if (s < thresh) break;
			end_x = u;
		}

		float gradient = */

		// Fill
		int end_x = 0;
		float end_d;
		for (int u=1; u<1000; ++u) {
			end_d = depth_in(x+u*dir, y);
			float s = smoothing(x+u*dir, y);
			if ((s < thresh) || (isValid(camera, end_d))) break;
			end_x = u;
		}

		float gradient = (end_d - value) / (float)end_x;

		for (int u=1; u<end_x; ++u) {
			depth_out(x+u*dir,y) = value;
			value += gradient;
		}
	}
}

void ftl::cuda::scan_field_fill(
		TextureObject<float> &depth_in,        // Virtual depth map
		TextureObject<float> &depth_out,   // Accumulated output
		TextureObject<float> &smoothing,
		float thresh,
		const ftl::rgbd::Camera &camera,
		cudaStream_t stream) {

	const dim3 gridSize(1, smoothing.height());
	const dim3 blockSize(128, 1);

	scan_field_fill_kernel<<<gridSize, blockSize, 0, stream>>>(depth_in, depth_out, smoothing, thresh, camera);
	cudaSafeCall( cudaGetLastError() );
}
