#include "mask_cuda.hpp"

#define T_PER_BLOCK 8

using ftl::cuda::Mask;

__global__ void discontinuity_kernel(ftl::cuda::TextureObject<int> mask_out,
										ftl::cuda::TextureObject<uchar4> support,
										ftl::cuda::TextureObject<float> depth, 
										const cv::Size size, const double minDepth, const double maxDepth,
										float threshold, int radius) {
	
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < size.width && y < size.height) {
		Mask mask(0);

		const float d = depth.tex2D((int)x, (int)y);

		if (d >= minDepth && d <= maxDepth) {
			/* Orts-Escolano S. et al. 2016. Holoportation: Virtual 3D teleportation in real-time. */

			// If colour cross support region terminates within the requested
			// radius, and the absolute depth difference on the other side is
			// greater than threshold, then is is a discontinuity.
			// Repeat for left, right, up and down.
			const uchar4 sup = support.tex2D((int)x, (int)y);
			if (sup.x <= radius) {
				float dS = depth.tex2D((int)x - sup.x - radius, (int)y);
				if (fabs(dS - d) > threshold) mask.isDiscontinuity(true);
			}
			if (sup.y <= radius) {
				float dS = depth.tex2D((int)x + sup.y + radius, (int)y);
				if (fabs(dS - d) > threshold) mask.isDiscontinuity(true);
			}
			if (sup.z <= radius) {
				float dS = depth.tex2D((int)x, (int)y - sup.z - radius);
				if (fabs(dS - d) > threshold) mask.isDiscontinuity(true);
			}
			if (sup.w <= radius) {
				float dS = depth.tex2D((int)x, (int)y + sup.w + radius);
				if (fabs(dS - d) > threshold) mask.isDiscontinuity(true);
			}
		}
		
		mask_out(x,y) = (int)mask;
	}
}

void ftl::cuda::discontinuity(	ftl::cuda::TextureObject<int> &mask_out, ftl::cuda::TextureObject<uchar4> &support,
								ftl::cuda::TextureObject<float> &depth,
								const cv::Size size, const double minDepth, const double maxDepth,
								int discon, float thresh, cudaStream_t stream) {
	
	const dim3 gridSize((size.width + T_PER_BLOCK - 1)/T_PER_BLOCK, (size.height + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	discontinuity_kernel<<<gridSize, blockSize, 0, stream>>>(mask_out, support, depth, size, minDepth, maxDepth, thresh, discon);
	cudaSafeCall( cudaGetLastError() );

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
#endif
}

__global__ void cull_discontinuity_kernel(ftl::cuda::TextureObject<int> mask, ftl::cuda::TextureObject<float> depth) {
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < depth.width() && y < depth.height()) {
		Mask m(mask.tex2D((int)x,(int)y));
		if (m.isDiscontinuity()) depth(x,y) = 1000.0f;
	}
}

void ftl::cuda::cull_discontinuity(ftl::cuda::TextureObject<int> &mask, ftl::cuda::TextureObject<float> &depth, cudaStream_t stream) {
	const dim3 gridSize((depth.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	cull_discontinuity_kernel<<<gridSize, blockSize, 0, stream>>>(mask, depth);
	cudaSafeCall( cudaGetLastError() );

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
#endif
}
