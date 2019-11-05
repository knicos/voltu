#include "smoothing_cuda.hpp"

#include <ftl/cuda/weighting.hpp>

using ftl::cuda::TextureObject;

#define T_PER_BLOCK 8

// ===== Colour-based Smooth ===================================================

template <int RADIUS>
__global__ void depth_smooth_kernel(
		ftl::cuda::TextureObject<float> depth_in,
		ftl::cuda::TextureObject<uchar4> colour_in,
		ftl::cuda::TextureObject<float> depth_out,
		ftl::rgbd::Camera camera,
		float factor, float thresh) {

	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < depth_in.width() && y < depth_in.height()) {
		float d = depth_in.tex2D((int)x,(int)y);
		depth_out(x,y) = 0.0f;

		if (d < camera.minDepth || d > camera.maxDepth) return;

		uchar4 c = colour_in.tex2D((int)x, (int)y);
		float3 pos = camera.screenToCam(x,y,d);

		float contrib = 0.0f;
		float new_depth = 0.0f;

		for (int v=-RADIUS; v<=RADIUS; ++v) {
			for (int u=-RADIUS; u<=RADIUS; ++u) {
				// Get colour difference to center
				const uchar4 cN = colour_in.tex2D((int)x+u, (int)y+v);
				const float colourWeight = ftl::cuda::colourWeighting(c, cN, thresh);
				const float dN = depth_in.tex2D((int)x + u, (int)y + v);
				const float3 posN = camera.screenToCam(x+u, y+v, dN);
				const float weight = ftl::cuda::spatialWeighting(posN, pos, factor * colourWeight);
				
				contrib += weight;
				new_depth += dN * weight;
			}
		}

		if (contrib > 0.0f) {
			depth_out(x,y) = new_depth / contrib;
		}
	}
}

void ftl::cuda::depth_smooth(
		ftl::cuda::TextureObject<float> &depth_in,
		ftl::cuda::TextureObject<uchar4> &colour_in,
		ftl::cuda::TextureObject<float> &depth_out,
		const ftl::rgbd::Camera &camera,
		int radius, float factor, float thresh, int iters, cudaStream_t stream) {

	const dim3 gridSize((depth_out.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth_out.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	for (int n=0; n<iters; ++n) {
		switch (radius) {
		case 5 :	depth_smooth_kernel<5><<<gridSize, blockSize, 0, stream>>>(depth_in, colour_in, depth_out, camera, factor, thresh); break;
		case 4 :	depth_smooth_kernel<4><<<gridSize, blockSize, 0, stream>>>(depth_in, colour_in, depth_out, camera, factor, thresh); break;
		case 3 :	depth_smooth_kernel<3><<<gridSize, blockSize, 0, stream>>>(depth_in, colour_in, depth_out, camera, factor, thresh); break;
		case 2 :	depth_smooth_kernel<2><<<gridSize, blockSize, 0, stream>>>(depth_in, colour_in, depth_out, camera, factor, thresh); break;
		case 1 :	depth_smooth_kernel<1><<<gridSize, blockSize, 0, stream>>>(depth_in, colour_in, depth_out, camera, factor, thresh); break;
		default:	break;
		}
		cudaSafeCall( cudaGetLastError() );

		switch (radius) {
		case 5 :	depth_smooth_kernel<5><<<gridSize, blockSize, 0, stream>>>(depth_out, colour_in, depth_in, camera, factor, thresh); break;
		case 4 :	depth_smooth_kernel<4><<<gridSize, blockSize, 0, stream>>>(depth_out, colour_in, depth_in, camera, factor, thresh); break;
		case 3 :	depth_smooth_kernel<3><<<gridSize, blockSize, 0, stream>>>(depth_out, colour_in, depth_in, camera, factor, thresh); break;
		case 2 :	depth_smooth_kernel<2><<<gridSize, blockSize, 0, stream>>>(depth_out, colour_in, depth_in, camera, factor, thresh); break;
		case 1 :	depth_smooth_kernel<1><<<gridSize, blockSize, 0, stream>>>(depth_out, colour_in, depth_in, camera, factor, thresh); break;
		default:	break;
		}
		cudaSafeCall( cudaGetLastError() );
	}

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
#endif
}

// ==== SMOOTHING FACTOR ==========

template <bool DERIV>
__device__ inline float getAverage(const ftl::rgbd::Camera &cam, float dd, const TextureObject<float> &d, int x, int y, int x1, int y1, int x2, int y2);

template <>
__device__ inline float getAverage<false>(const ftl::rgbd::Camera &cam, float dd, const TextureObject<float> &d, int x, int y, int x1, int y1, int x2, int y2) {
	float a = d.tex2D(x+x1,y+y1);
	float b = d.tex2D(x+x2,y+y2);
	return (a <= cam.minDepth || a > cam.maxDepth || b <= cam.minDepth || b > cam.maxDepth) ? dd : (a+b) / 2.0f;
}

template <>
__device__ inline float getAverage<true>(const ftl::rgbd::Camera &cam, float dd, const TextureObject<float> &d, int x, int y, int x1, int y1, int x2, int y2) {
	float a = d.tex2D(x+x1,y+y1);
	float b = d.tex2D(x+x2,y+y2);
	return (a+b) / 2.0f;
}

__device__ inline void absmin(float &minvar, float v) {
	if (fabs(minvar) > fabs(v)) minvar = v;
}

template <bool DERIV>
__global__ void smoothing_factor_kernel(
		ftl::cuda::TextureObject<float> depth_in,
		//ftl::cuda::TextureObject<uchar4> colour_in,
		ftl::cuda::TextureObject<float> smoothing,
		//float thresh,
		ftl::rgbd::Camera camera) {

	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < depth_in.width() && y < depth_in.height()) {
		float d = depth_in.tex2D((int)x,(int)y);

		//if (d < camera.minDepth || d > camera.maxDepth) return;

		float min_var = 10.0f;
		float max_var = 0.0f;

		float avg = 0.0f;
		float var;

		var = (d - getAverage<DERIV>(camera, d, depth_in, x, y, -1, -1, 1, 1));
		//avg += var;
		absmin(min_var, var);
		var = (d - getAverage<DERIV>(camera, d, depth_in, x, y, 0, -1, 0, 1));
		//avg += var;
		absmin(min_var, var);
		var = (d - getAverage<DERIV>(camera, d, depth_in, x, y, 1, -1, -1, 1));
		//avg += var;
		absmin(min_var, var);
		var = (d - getAverage<DERIV>(camera, d, depth_in, x, y, -1, 0, 1, 0));
		//avg += var;
		absmin(min_var, var);

		// Clamp to threshold
		//min_var = min(min_var, thresh);
		//float s = 1.0f - (min_var / thresh);
		smoothing(x,y) = min_var;
	}
}

__global__ void norm_thresh_kernel(
		ftl::cuda::TextureObject<float> in,
		ftl::cuda::TextureObject<float> error_value,
		ftl::cuda::TextureObject<float> out,
		float thresh) {

	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < in.width() && y < in.height()) {
		// Clamp to threshold
		float min_var = min(in.tex2D((int)x,(int)y), thresh);
		float s = min(1.0f, (fabs(min_var) / thresh));
		out(x,y) = s * error_value(x,y);
	}
}

__global__ void do_smooth_kernel(
		ftl::cuda::TextureObject<float> smooth_strength,
		//ftl::cuda::TextureObject<float> error_value,
		ftl::cuda::TextureObject<float> depth) {

	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < depth.width() && y < depth.height()) {
		depth(x,y) = depth(x,y) - smooth_strength(x,y);
	}
}

template <int RADIUS>
__global__ void sum_neighbors_kernel(
		ftl::cuda::TextureObject<float> depth_in,
		ftl::cuda::TextureObject<float> depth_out,
		ftl::rgbd::Camera camera, float alpha) {

	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < depth_out.width() && y < depth_out.height()) {
		float avg = 0.0f;
		float contrib = 0.0f;

		float d0 = depth_in.tex2D((int)x, (int)y);
		float3 pos0 = camera.screenToCam(x,y,d0);

		for (int v=-RADIUS; v<=RADIUS; ++v) {
			#pragma unroll
			for (int u=-RADIUS; u<=RADIUS; ++u) {
				float dN = depth_in.tex2D((int)x + u, (int)y + v);
				float3 posN = camera.screenToCam(x+u,y+v,dN);
				float weight = ftl::cuda::spatialWeighting(pos0, posN, alpha);
				avg += weight * dN;
				contrib += weight;
			}
		}

		depth_out(x,y) = avg / contrib;
	}
}

void ftl::cuda::smoothing_factor(
		ftl::cuda::TextureObject<float> &depth_in,
		//ftl::cuda::TextureObject<float> &depth_tmp,
		ftl::cuda::TextureObject<float> &temp,
		//ftl::cuda::TextureObject<uchar4> &colour_in,
		ftl::cuda::TextureObject<float> &smoothing,
		float thresh,
		const ftl::rgbd::Camera &camera,
		cudaStream_t stream) {

	const dim3 gridSize((smoothing.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (smoothing.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	//smoothing_factor_kernel<<<gridSize, blockSize, 0, stream>>>(depth_in, temp, camera);

	//float thresh2 = thresh;
	//float alpha = 0.04f;

	//for (int i=0; i<10; ++i) {
		
		smoothing_factor_kernel<false><<<gridSize, blockSize, 0, stream>>>(depth_in, temp, camera);
		smoothing_factor_kernel<true><<<gridSize, blockSize, 0, stream>>>(temp, smoothing, camera);
		norm_thresh_kernel<<<gridSize, blockSize, 0, stream>>>(smoothing, temp, smoothing, thresh);
		do_smooth_kernel<<<gridSize, blockSize, 0, stream>>>(smoothing, depth_in);

		//do_smooth_kernel<<<gridSize, blockSize, 0, stream>>>(smoothing, bufs[(ix+1)%2], bufs[ix%2]);
		//if (i == 0) sum_neighbors_kernel<1><<<gridSize, blockSize, 0, stream>>>(depth_in, bufs[ix%2], camera, alpha);
		//else {
		//	sum_neighbors_kernel<2><<<gridSize, blockSize, 0, stream>>>(bufs[ix%2], bufs[(ix+1)%2], camera, alpha);
		//	++ix;
		//}
		//thresh2 *= 2.0f;
		//alpha *= 3.0f;
	//}

	//sum_neighbors_kernel<<<gridSize, blockSize, 0, stream>>>(depth_in, temp);

	cudaSafeCall( cudaGetLastError() );


	#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	#endif
}
