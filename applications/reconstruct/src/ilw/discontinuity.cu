#include "ilw_cuda.hpp"

#define T_PER_BLOCK 8

using ftl::cuda::Mask;

template <int RADIUS>
__global__ void discontinuity_kernel(ftl::cuda::TextureObject<int> mask_out, ftl::cuda::TextureObject<float> depth,
										const cv::Size size, const double minDepth, const double maxDepth) {
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < size.width && y < size.height) {
		Mask mask(0);

		const float d = depth.tex2D((int)x, (int)y);

		// Calculate depth between 0.0 and 1.0
		//float p = (d - params.minDepth) / (params.maxDepth - params.minDepth);

		if (d >= minDepth && d <= maxDepth) {
			/* Orts-Escolano S. et al. 2016. Holoportation: Virtual 3D teleportation in real-time. */
			// Is there a discontinuity nearby?
			for (int u=-RADIUS; u<=RADIUS; ++u) {
				for (int v=-RADIUS; v<=RADIUS; ++v) {
					// If yes, the flag using w = -1
					if (fabs(depth.tex2D((int)x+u, (int)y+v) - d) > 0.1f) mask.isDiscontinuity(true);
				}
			}
		}
		
		mask_out(x,y) = (int)mask;
	}
}

void ftl::cuda::discontinuity(ftl::cuda::TextureObject<int> &mask_out, ftl::cuda::TextureObject<float> &depth,
								const cv::Size size, const double minDepth, const double maxDepth,
								uint discon, cudaStream_t stream) {
			
	const dim3 gridSize((size.width + T_PER_BLOCK - 1)/T_PER_BLOCK, (size.height + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	switch (discon) {
	case 5 :	discontinuity_kernel<5><<<gridSize, blockSize, 0, stream>>>(mask_out, depth, size, minDepth, maxDepth); break;
	case 4 :	discontinuity_kernel<4><<<gridSize, blockSize, 0, stream>>>(mask_out, depth, size, minDepth, maxDepth); break;
	case 3 :	discontinuity_kernel<3><<<gridSize, blockSize, 0, stream>>>(mask_out, depth, size, minDepth, maxDepth); break;
	case 2 :	discontinuity_kernel<2><<<gridSize, blockSize, 0, stream>>>(mask_out, depth, size, minDepth, maxDepth); break;
	case 1 :	discontinuity_kernel<1><<<gridSize, blockSize, 0, stream>>>(mask_out, depth, size, minDepth, maxDepth); break;
	default:	break;
	}
	cudaSafeCall( cudaGetLastError() );

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
#endif
}
