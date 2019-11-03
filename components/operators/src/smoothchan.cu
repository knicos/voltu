#include "smoothing_cuda.hpp"

#include <ftl/cuda/weighting.hpp>

using ftl::cuda::TextureObject;

#define T_PER_BLOCK 8

template <int RADIUS>
__global__ void smooth_chan_kernel(
		ftl::cuda::TextureObject<uchar4> colour_in,
		//ftl::cuda::TextureObject<float> depth_in,
		ftl::cuda::TextureObject<float> smoothing_out,
		ftl::rgbd::Camera camera,
		float alpha, float scale) {

	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < smoothing_out.width() && y < smoothing_out.height()) {
		const int ix = int(((float)x / (float)smoothing_out.width()) * (float)colour_in.width());
		const int iy = int(((float)y / (float)smoothing_out.height()) * (float)colour_in.height());

		// A distance has already been found
		if (smoothing_out(x,y) < 1.0f) return;

		float mindist = 100.0f;

		const uchar4 c0 = colour_in.tex2D(ix, iy);
		//const float d0 = depth_in.tex2D(ix, iy);
		//if (d0 < camera.minDepth || d0 > camera.maxDepth) return;

		//const float3 pos = camera.screenToCam(ix, iy, d0);

		for (int v=-RADIUS; v<=RADIUS; ++v) {
			#pragma unroll
			for (int u=-RADIUS; u<=RADIUS; ++u) {
				const uchar4 c = colour_in.tex2D(ix+u, iy+v);
				//const float d = depth_in.tex2D(ix+u, iy+v);
				//if (d < camera.minDepth || d > camera.maxDepth) continue;
				//const float3 posN = camera.screenToCam(ix+u, iy+v, d);

				float d = sqrt((float)u*(float)u + (float)v*(float)v) * scale;

				if (ftl::cuda::colourDistance(c, c0) >= alpha) mindist = min(mindist, (float)d);
			}
		}

		smoothing_out(x,y) = min(mindist / 100.0f, 1.0f);
	}
}

void ftl::cuda::smooth_channel(
		ftl::cuda::TextureObject<uchar4> &colour_in,
		//ftl::cuda::TextureObject<float> &depth_in,
		ftl::cuda::TextureObject<float> &smoothing_out,
		const ftl::rgbd::Camera &camera,
		float alpha,
		float scale,
		int radius,
		cudaStream_t stream) {

	const dim3 gridSize((smoothing_out.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (smoothing_out.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);


	switch (radius) {
		case 5: smooth_chan_kernel<5><<<gridSize, blockSize, 0, stream>>>(colour_in, smoothing_out, camera, alpha, scale); break;
		case 4: smooth_chan_kernel<4><<<gridSize, blockSize, 0, stream>>>(colour_in, smoothing_out, camera, alpha, scale); break;
		case 3: smooth_chan_kernel<3><<<gridSize, blockSize, 0, stream>>>(colour_in, smoothing_out, camera, alpha, scale); break;
		case 2: smooth_chan_kernel<2><<<gridSize, blockSize, 0, stream>>>(colour_in, smoothing_out, camera, alpha, scale); break;
		case 1: smooth_chan_kernel<1><<<gridSize, blockSize, 0, stream>>>(colour_in, smoothing_out, camera, alpha, scale); break;
	}
		

	cudaSafeCall( cudaGetLastError() );


	#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	#endif
}
