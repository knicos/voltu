#include "smoothing.hpp"

#include <ftl/cuda/weighting.hpp>

#define T_PER_BLOCK 8

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
