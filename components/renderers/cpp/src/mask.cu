#include "splatter_cuda.hpp"
#include <ftl/cuda/mask.hpp>

using ftl::cuda::TextureObject;
using ftl::cuda::Mask;

#define T_PER_BLOCK 16

__global__ void show_mask_kernel(
        TextureObject<uchar4> colour,
        TextureObject<int> mask,
        int id, uchar4 style) {
        
	const int x = (blockIdx.x*blockDim.x + threadIdx.x);
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x >= 0 && x < colour.width() && y >=0 && y < colour.height()) {
        Mask m(mask.tex2D(x,y));

        if (m.is(id)) {
            colour(x,y) = style;
        }
    }
}


void ftl::cuda::show_mask(
        TextureObject<uchar4> &colour,
        TextureObject<int> &mask,
        int id, uchar4 style, cudaStream_t stream) {
	const dim3 gridSize((colour.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (colour.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    show_mask_kernel<<<gridSize, blockSize, 0, stream>>>(
        colour, mask, id, style
    );
    cudaSafeCall( cudaGetLastError() );
}
