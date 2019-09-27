#include "ilw_cuda.hpp"

using ftl::cuda::TextureObject;
using ftl::rgbd::Camera;

#define WARP_SIZE 32
#define T_PER_BLOCK 8
#define FULL_MASK 0xffffffff

__device__ inline float warpMax(float e) {
	for (int i = WARP_SIZE/2; i > 0; i /= 2) {
		const float other = __shfl_xor_sync(FULL_MASK, e, i, WARP_SIZE);
		e = max(e, other);
	}
	return e;
}

__global__ void correspondence_energy_vector_kernel(
        TextureObject<float4> p1,
        TextureObject<float4> p2,
        TextureObject<uchar4> c1,
        TextureObject<uchar4> c2,
        TextureObject<float4> vout,
        TextureObject<float> eout,
        float4x4 pose2,  // Inverse
        Camera cam2) {

    // Each warp picks point in p1
    const int tid = (threadIdx.x + threadIdx.y * blockDim.x);
	const int x = (blockIdx.x*blockDim.x + threadIdx.x) / WARP_SIZE;
    const int y = blockIdx.y*blockDim.y + threadIdx.y;
    
    const float3 world1 = make_float3(p1.tex2D(x, y));
    const float3 camPos2 = pose2 * world1;
    const uint2 screen2 = cam2.camToScreen<uint2>(camPos2);

    const int upsample = 8;

    // Project to p2 using cam2
    // Each thread takes a possible correspondence and calculates a weighting
    const int lane = tid % WARP_SIZE;
	for (int i=lane; i<upsample*upsample; i+=WARP_SIZE) {
		const float u = (i % upsample) - (upsample / 2);
        const float v = (i / upsample) - (upsample / 2);
        
        const float3 world2 = make_float3(p2.tex2D(screen2.x+u, screen2.y+v));

        // Determine degree of correspondence
        const float confidence = 1.0f / length(world1 - world2);

        printf("conf %f\n", confidence);
        const float maxconf = warpMax(confidence);

        // This thread has best confidence value
        if (maxconf == confidence) {
            vout(x,y) = vout.tex2D(x, y) + make_float4(
                (world1.x - world2.x) * maxconf,
                (world1.y - world2.y) * maxconf,
                (world1.z - world2.z) * maxconf,
                maxconf);
            eout(x,y) = eout.tex2D(x,y) + length(world1 - world2)*maxconf;
        }
    }
}

void ftl::cuda::correspondence_energy_vector(
        TextureObject<float4> &p1,
        TextureObject<float4> &p2,
        TextureObject<uchar4> &c1,
        TextureObject<uchar4> &c2,
        TextureObject<float4> &vout,
        TextureObject<float> &eout,
        float4x4 &pose2,
        const Camera &cam2,
        cudaStream_t stream) {

    const dim3 gridSize((p1.width() + 2 - 1)/2, (p1.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(2*WARP_SIZE, T_PER_BLOCK);

    printf("COR SIZE %d,%d\n", p1.width(), p1.height());

    correspondence_energy_vector_kernel<<<gridSize, blockSize, 0, stream>>>(
        p1, p2, c1, c2, vout, eout, pose2, cam2
    );
    cudaSafeCall( cudaGetLastError() );
}
