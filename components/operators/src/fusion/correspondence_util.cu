#include "mvmls_cuda.hpp"
#include <ftl/cuda/weighting.hpp>
#include <ftl/operators/cuda/mask.hpp>
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
		TextureObject<short2> screen2,
		float thresh) {

	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < colour.width() && y < colour.height()) {
		short2 s1 = screen1.tex2D(x,y);

		//colour(x,y) = make_uchar4(0,0,0,0);

		if (s1.x >= 0 && s1.x < screen2.width() && s1.y < screen2.height()) {
			short2 s2 = screen2.tex2D(s1.x, s1.y);

			float l = sqrt(square(s2.x-x) + square(s2.y-y));
			float nl = min(1.0f, l/thresh);
			//colour(x,y) = (l < 1.0f) ? make_uchar4(0,255,0,0) : (s2.x < 0) ? make_uchar4(255.0f, 0.0f, 0.0f, 0.0f) : make_uchar4(0.0f, (1.0f-nl)*255.0f, nl*255.0f, 0.0f);
			if (nl < 1.0f && s2.x >= 0) colour(x,y) = make_uchar4(0.0f, (1.0f-nl)*255.0f, nl*255.0f, 0.0f);
		}
	}
}

void ftl::cuda::show_cor_error(TextureObject<uchar4> &colour, TextureObject<short2> &screen1, TextureObject<short2> &screen2, float thresh, cudaStream_t stream) {
	const dim3 gridSize((colour.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (colour.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	show_cor_error_kernel<<<gridSize, blockSize, 0, stream>>>(colour, screen1, screen2, thresh);
	cudaSafeCall( cudaGetLastError() );
}

// ==== Remove correspondence errors ===========================================

__global__ void remove_cor_error_kernel(
		TextureObject<float> adjust,
		TextureObject<short2> screen1,
		TextureObject<short2> screen2, float thresh) {

	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < adjust.width() && y < adjust.height()) {
		short2 s1 = screen1.tex2D(x,y);

		if (s1.x >= 0 && s1.x < screen2.width() && s1.y < screen2.height()) {
			short2 s2 = screen2.tex2D(s1.x, s1.y);

			float l = sqrt(square(s2.x-x) + square(s2.y-y));
			if (l >= thresh || s2.x < 0) {
				adjust(x,y) = PINF;
				screen1(x,y) = make_short2(-1,-1);
			}
		}
	}
}

void ftl::cuda::remove_cor_error(TextureObject<float> &adjust, TextureObject<short2> &screen1, TextureObject<short2> &screen2, float thresh, cudaStream_t stream) {
	const dim3 gridSize((adjust.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (adjust.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	remove_cor_error_kernel<<<gridSize, blockSize, 0, stream>>>(adjust, screen1, screen2, thresh);
	cudaSafeCall( cudaGetLastError() );
}


// ==== Show depth adjustments =================================================

__global__ void show_depth_adjust_kernel(
		TextureObject<uchar4> colour,
		TextureObject<short2> screen,
		TextureObject<float> adjust, float scale) {

	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < colour.width() && y < colour.height()) {
		float a = adjust.tex2D(x,y);
		short2 s = screen.tex2D(x,y);

		//colour(x,y) = make_uchar4(0,0,0,0);

		if (s.x >= 0) {
			float ncG = min(1.0f, fabsf(a)/scale);
			float ncB = -max(-1.0f, min(0.0f, a/scale));
			float ncR = max(0.0f, min(1.0f, a/scale));
			colour(x,y) = make_uchar4(ncB*255.0f, (1.0f-ncG)*255.0f, ncR*255.0f, 0.0f);
		}
	}
}

void ftl::cuda::show_depth_adjustment(TextureObject<uchar4> &colour, TextureObject<short2> &screen, TextureObject<float> &adjust, float scale, cudaStream_t stream) {
	const dim3 gridSize((colour.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (colour.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	show_depth_adjust_kernel<<<gridSize, blockSize, 0, stream>>>(colour, screen, adjust, scale);
	cudaSafeCall( cudaGetLastError() );
}

// ==== Vis reprojection =======================================================

__global__ void viz_reprojection_kernel(
		TextureObject<uchar4> colour_out,
		TextureObject<uchar4> colour_in,
		TextureObject<float> depth_out,
		TextureObject<float> depth_in,
		float4x4 pose,
		Camera cam1,
		Camera cam2) {

	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < colour_in.width() && y < colour_in.height()) {
		float d = depth_in.tex2D(x,y);
		const float3 camPosOrigin = pose * cam1.screenToCam(x,y,d);
        const uint2 p = cam2.camToScreen<uint2>(camPosOrigin);

		if (p.x < colour_out.width() && p.y < colour_out.height()) {
			float dout = depth_out(p.x, p.y);
			uchar4 cin = colour_in(x,y);
			uchar4 cout = colour_out(p.x, p.y);

			if (fabs(dout-camPosOrigin.z) < 0.1f) {
				colour_out(p.x, p.y) = make_uchar4(
					(int(cin.x)+int(cout.x)) / 2,
					(int(cin.y)+int(cout.y)) / 2,
					(int(cin.z)+int(cout.z)) / 2,
					0);
			}
		}
	}
}

void ftl::cuda::viz_reprojection(
		TextureObject<uchar4> &colour_out,
		TextureObject<uchar4> &colour_in,
		TextureObject<float> &depth_out,
		TextureObject<float> &depth_in,
		const float4x4 &pose,
		const Camera &cam1,
		const Camera &cam2, cudaStream_t stream) {
	const dim3 gridSize((colour_in.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (colour_in.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	viz_reprojection_kernel<<<gridSize, blockSize, 0, stream>>>(colour_out, colour_in, depth_out, depth_in, pose, cam1, cam2);
	cudaSafeCall( cudaGetLastError() );
}
