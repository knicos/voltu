#include "ilw_cuda.hpp"
#include <ftl/cuda/weighting.hpp>

using ftl::cuda::Mask;

#define T_PER_BLOCK 8

template <int RADIUS>
__global__ void preprocess_kernel(
    	ftl::cuda::TextureObject<float> depth_in,
		ftl::cuda::TextureObject<float> depth_out,
		ftl::cuda::TextureObject<uchar4> colour,
		ftl::cuda::TextureObject<int> mask,
		ftl::rgbd::Camera camera,
		ftl::cuda::ILWParams params) {

    const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

    float d = depth_in.tex2D((int)x,(int)y);
    Mask main_mask(mask.tex2D((int)x,(int)y));
	uchar4 c = colour.tex2D((int)x,(int)y);

	// Calculate discontinuity mask

	// Fill missing depths
	if (d < camera.minDepth || d > camera.maxDepth) {
		float depth_accum = 0.0f;
		float contrib = 0.0f;

        int v=0;
		//for (int v=-RADIUS; v<=RADIUS; ++v) {
			for (int u=-RADIUS; u<=RADIUS; ++u) {
				uchar4 c2 = colour.tex2D((int)x+u,(int)y+v);
                float d2 = depth_in.tex2D((int)x+u,(int)y+v);
                Mask m(mask.tex2D((int)x+u,(int)y+v));

				if (!m.isDiscontinuity() && d2 >= camera.minDepth && d2 <= camera.maxDepth) {
					float w = ftl::cuda::colourWeighting(c, c2, params.fill_match);
					depth_accum += d2*w;
					contrib += w;
				}
			}
		//}

		if (contrib > params.fill_threshold) {
            d = depth_accum / contrib;
            main_mask.isFilled(true);
        }
	}

    mask(x,y) = (int)main_mask;
	depth_out(x,y) = d;
}

void ftl::cuda::preprocess_depth(
		ftl::cuda::TextureObject<float> &depth_in,
		ftl::cuda::TextureObject<float> &depth_out,
		ftl::cuda::TextureObject<uchar4> &colour,
		ftl::cuda::TextureObject<int> &mask,
		const ftl::rgbd::Camera &camera,
		const ftl::cuda::ILWParams &params,
		cudaStream_t stream) {

	const dim3 gridSize((depth_in.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth_in.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	preprocess_kernel<10><<<gridSize, blockSize, 0, stream>>>(depth_in, depth_out, colour, mask, camera, params);

	cudaSafeCall( cudaGetLastError() );
}
