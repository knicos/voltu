#include "mvmls_cuda.hpp"
#include <ftl/cuda/weighting.hpp>
#include <ftl/operators/mask_cuda.hpp>
#include <ftl/cuda/warp.hpp>

using ftl::cuda::TextureObject;
using ftl::rgbd::Camera;
using ftl::cuda::Mask;
using ftl::cuda::MvMLSParams;

#define T_PER_BLOCK 8
#define WARP_SIZE 32

#include "correspondence_common.hpp"

// ==== Remove zero-confidence =================================================

__global__ void zero_confidence_kernel(
		TextureObject<float> conf,
		TextureObject<float> depth) {

	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < depth.width() && y < depth.height()) {
		const float c = conf.tex2D((int)x,(int)y);

		if (c == 0.0f) {
			depth(x,y) = 1000.0f;	
		}
	}
}

void ftl::cuda::zero_confidence(TextureObject<float> &conf, TextureObject<float> &depth, cudaStream_t stream) {
	const dim3 gridSize((depth.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	zero_confidence_kernel<<<gridSize, blockSize, 0, stream>>>(conf, depth);
	cudaSafeCall( cudaGetLastError() );
}

// ==== Show correspondence errors =============================================

__global__ void show_cor_error_kernel(
		TextureObject<uchar4> colour,
		TextureObject<short2> screen1,
		TextureObject<short2> screen2) {

	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < colour.width() && y < colour.height()) {
		short2 s1 = screen1.tex2D(x,y);

		if (s1.x >= 0 && s1.x < screen2.width() && s1.y < screen2.height()) {
			short2 s2 = screen2.tex2D(s1.x, s1.y);

			float l = sqrt(square(s2.x-x) + square(s2.y-y));
			float nl = min(1.0f, l/5.0f);
			//colour(x,y) = (l < 1.0f) ? make_uchar4(0,255,0,0) : (s2.x < 0) ? make_uchar4(255.0f, 0.0f, 0.0f, 0.0f) : make_uchar4(0.0f, (1.0f-nl)*255.0f, nl*255.0f, 0.0f);
			if (nl < 1.0f && s2.x >= 0) colour(x,y) = make_uchar4(0.0f, (1.0f-nl)*255.0f, nl*255.0f, 0.0f);
		}
	}
}

void ftl::cuda::show_cor_error(TextureObject<uchar4> &colour, TextureObject<short2> &screen1, TextureObject<short2> &screen2, cudaStream_t stream) {
	const dim3 gridSize((colour.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (colour.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	show_cor_error_kernel<<<gridSize, blockSize, 0, stream>>>(colour, screen1, screen2);
	cudaSafeCall( cudaGetLastError() );
}


// ==== Show depth adjustments =================================================

__global__ void show_depth_adjust_kernel(
		TextureObject<uchar4> colour,
		TextureObject<float> adjust) {

	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < colour.width() && y < colour.height()) {
		float a = adjust.tex2D(x,y);

		if (a != 0.0f) {
			float nc = min(1.0f, fabsf(a)/0.04f);
			colour(x,y) = make_uchar4(0.0f, (1.0f-nc)*255.0f, nc*255.0f, 0.0f);
		}
	}
}

void ftl::cuda::show_depth_adjustment(TextureObject<uchar4> &colour, TextureObject<float> &adjust, cudaStream_t stream) {
	const dim3 gridSize((colour.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (colour.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	show_depth_adjust_kernel<<<gridSize, blockSize, 0, stream>>>(colour, adjust);
	cudaSafeCall( cudaGetLastError() );
}
