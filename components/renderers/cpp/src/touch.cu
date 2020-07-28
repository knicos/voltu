#include <ftl/cuda/touch.hpp>
#include <ftl/cuda/warp.hpp>

using ftl::cuda::TextureObject;
using ftl::cuda::warpSum;

__device__ inline ftl::cuda::Collision pack_collision(int cx, int cy, int num, float cd) {
    return ftl::cuda::Collision{(num << 24) | (cx << 12) | (cy), cd};
}

 __global__ void touch_kernel(TextureObject<float> depth_in, TextureObject<float> depth_out, ftl::cuda::Collision *collisions, int max_collisions, float dist) {
	const int x = blockIdx.x*blockDim.x + threadIdx.x;
    const int y = blockIdx.y*blockDim.y + threadIdx.y;

	bool collision = false;
	float cd = 0.0f;

    if (x >= 0 && y >= 0 && x < depth_in.width() && y < depth_in.height()) {
		//uint2 screenPos = make_uint2(30000,30000);

		const float din = depth_in.tex2D(x, y);
        const float dout = depth_out.tex2D(x, y);

		collision = (din < 1000.0f && fabsf(din-dout) < dist);
		cd = fminf(din,dout);
        depth_out(x,y) = cd;
    }
    
    int num_collisions = __popc(__ballot_sync(0xFFFFFFFF, collision));
    float cx = warpSum((collision) ? float(x) : 0.0f) / float(num_collisions);
	float cy = warpSum((collision) ? float(y) : 0.0f) / float(num_collisions);
	cd = warpSum((collision) ? float(cd) : 0.0f) / float(num_collisions);
    if ((threadIdx.x+threadIdx.y*blockDim.x) % 32 == 0) {
        if (num_collisions > 0) {
            //printf("Collision: %f,%f [%d]\n", cx, cy, num_collisions);
            int ix = atomicInc(&collisions[0].screen, max_collisions-1);
            collisions[ix+1] = pack_collision(cx, cy, num_collisions, cd);
        }
    }
}

#define T_PER_BLOCK 8

void ftl::cuda::touch_merge(TextureObject<float> &depth_in, TextureObject<float> &depth_out, ftl::cuda::Collision *collisions, int max_collisions, float dist, cudaStream_t stream) {
    const dim3 gridSize((depth_in.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth_in.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    touch_kernel<<<gridSize, blockSize, 0, stream>>>(depth_in, depth_out, collisions, max_collisions, dist);
	cudaSafeCall( cudaGetLastError() );
}
