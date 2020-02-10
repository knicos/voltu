#include <ftl/operators/mask_cuda.hpp>

#define T_PER_BLOCK 8

using ftl::cuda::Mask;

/* OLD VERSION */
__global__ void discontinuity_kernel(ftl::cuda::TextureObject<uint8_t> mask_out,
										ftl::cuda::TextureObject<uchar4> support,
										ftl::cuda::TextureObject<float> depth, 
										const cv::Size size, const double minDepth, const double maxDepth,
										float depthCoef, int radius) {
	
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < size.width && y < size.height) {
		Mask mask(0);

		const float d = depth.tex2D((int)x, (int)y);
		// Multiples of pixel size at given depth
		//const float threshold = (depthCoef / ((depthCoef / d) - (radius+disconDisparities-1))) - d;
		const float threshold = depthCoef * d;  // Where depthCoef = 1 / focal * N, N = number of pixel distances equal to a discon.

		if (d > minDepth && d < maxDepth) {
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

			// FIXME: The above results in a cross formation, need to test all 8 directions
		}
		
		mask_out(x,y) = (int)mask;
	}
}

/* New / Current version */
__global__ void discontinuity_kernel(ftl::cuda::TextureObject<uint8_t> mask_out,
		ftl::cuda::TextureObject<uchar4> support,
		ftl::cuda::TextureObject<float> depth, 
		const cv::Size size, const double minDepth, const double maxDepth,
		float depthCoef, float discon_thresh, float noise_thresh, float area_max) {

	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < size.width && y < size.height) {
		Mask mask(0);

		const float d = depth.tex2D((int)x, (int)y);
		// Multiples of pixel size at given depth
		//const float threshold = (depthCoef / ((depthCoef / d) - (radius+disconDisparities-1))) - d;
		const float threshold = depthCoef * d;  // Where depthCoef = 1 / focal * N, N = number of pixel distances equal to a discon.

		if (d > minDepth && d < maxDepth) {
			/* Orts-Escolano S. et al. 2016. Holoportation: Virtual 3D teleportation in real-time.
			 * This paper just says to remove values around discontinuities. */

			// Find max change in depth gradient in each direction
			const float g1 = fabsf((depth.tex2D(x-1, y) - d) - (d - depth.tex2D(x+1,y)));
			const float g2 = fabsf((depth.tex2D(x, y-1) - d) - (d - depth.tex2D(x,y+1)));
			const float g3 = fabsf((depth.tex2D(x-1, y-1) - d) - (d - depth.tex2D(x+1,y+1)));
			const float g4 = fabsf((depth.tex2D(x+1, y-1) - d) - (d - depth.tex2D(x-1,y+1)));
			const float g = max(g1,max(g2,(max(g3,g4))));

			// Calculate support window area
			const uchar4 sup = support.tex2D((int)x, (int)y);
			const float supx = min(sup.x,sup.y);
			const float supy = min(sup.z,sup.w);
			const float area = supx * supy;

			float grad_weight = min(1.0f, g / threshold);
			float area_weight = min(1.0f, area / area_max);

			if (grad_weight * (1.0f - area_weight) > discon_thresh) mask.isDiscontinuity(true);
			if (grad_weight * (area_weight) > noise_thresh) mask.isNoise(true);
		}

		mask_out(x,y) = (int)mask;
	}
}

void ftl::cuda::discontinuity(	ftl::cuda::TextureObject<uint8_t> &mask_out, ftl::cuda::TextureObject<uchar4> &support,
								ftl::cuda::TextureObject<float> &depth,
								const cv::Size size, const double minDepth, const double maxDepth,
								int discon, float depthCoef, cudaStream_t stream) {
	
	const dim3 gridSize((size.width + T_PER_BLOCK - 1)/T_PER_BLOCK, (size.height + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	discontinuity_kernel<<<gridSize, blockSize, 0, stream>>>(mask_out, support, depth, size, minDepth, maxDepth, depthCoef, discon);
	cudaSafeCall( cudaGetLastError() );

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
#endif
}

void ftl::cuda::discontinuity(	ftl::cuda::TextureObject<uint8_t> &mask_out, ftl::cuda::TextureObject<uchar4> &support,
		ftl::cuda::TextureObject<float> &depth,
		const cv::Size size, const double minDepth, const double maxDepth,
		float depthCoef, float discon_thresh, float noise_thresh, float area_max, cudaStream_t stream) {

	const dim3 gridSize((size.width + T_PER_BLOCK - 1)/T_PER_BLOCK, (size.height + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	discontinuity_kernel<<<gridSize, blockSize, 0, stream>>>(mask_out, support, depth, size, minDepth, maxDepth, depthCoef, discon_thresh, noise_thresh, area_max);
	cudaSafeCall( cudaGetLastError() );

	#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	#endif
}

// =============================================================================

template <int RADIUS>
__global__ void cull_mask_kernel(ftl::cuda::TextureObject<uint8_t> mask, ftl::cuda::TextureObject<float> depth, uint8_t id) {
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < depth.width()-RADIUS && y < depth.height()-RADIUS) {
		bool isdiscon = false;

		#pragma unroll
		for (int v=-RADIUS; v<=RADIUS; ++v) {
		#pragma unroll
		for (int u=-RADIUS; u<=RADIUS; ++u) {
			Mask m(mask.tex2D((int)x+u,(int)y+v));
			isdiscon = isdiscon || m.is(id);
		}
		}

		if (isdiscon) {
			depth(x,y) = 0.0f;
		}
	}
}

void ftl::cuda::cull_mask(ftl::cuda::TextureObject<uint8_t> &mask, ftl::cuda::TextureObject<float> &depth, uint8_t id, unsigned int radius, cudaStream_t stream) {
	const dim3 gridSize((depth.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	switch (radius) {
	case 0	: cull_mask_kernel<0><<<gridSize, blockSize, 0, stream>>>(mask, depth, id); break;
	case 1	: cull_mask_kernel<1><<<gridSize, blockSize, 0, stream>>>(mask, depth, id); break;
	case 2	: cull_mask_kernel<2><<<gridSize, blockSize, 0, stream>>>(mask, depth, id); break;
	case 3	: cull_mask_kernel<3><<<gridSize, blockSize, 0, stream>>>(mask, depth, id); break;
	case 4	: cull_mask_kernel<4><<<gridSize, blockSize, 0, stream>>>(mask, depth, id); break;
	case 5	: cull_mask_kernel<5><<<gridSize, blockSize, 0, stream>>>(mask, depth, id); break;
	default: break;
	}
	cudaSafeCall( cudaGetLastError() );

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
#endif
}
