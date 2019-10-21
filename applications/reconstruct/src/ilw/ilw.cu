#include "ilw_cuda.hpp"
#include <ftl/cuda/weighting.hpp>

using ftl::cuda::TextureObject;
using ftl::rgbd::Camera;

#define WARP_SIZE 32
#define T_PER_BLOCK 8
#define FULL_MASK 0xffffffff

__device__ inline float warpMin(float e) {
	for (int i = WARP_SIZE/2; i > 0; i /= 2) {
		const float other = __shfl_xor_sync(FULL_MASK, e, i, WARP_SIZE);
		e = min(e, other);
	}
	return e;
}

__device__ inline float warpSum(float e) {
	for (int i = WARP_SIZE/2; i > 0; i /= 2) {
		const float other = __shfl_xor_sync(FULL_MASK, e, i, WARP_SIZE);
		e += other;
	}
	return e;
}

//==============================================================================



//==============================================================================



//==============================================================================

//#define MOTION_RADIUS 9

template <int MOTION_RADIUS>
__global__ void move_points_kernel(
    ftl::cuda::TextureObject<float> d_old,
	ftl::cuda::TextureObject<float> d_new,
	ftl::cuda::TextureObject<float> conf,
    ftl::rgbd::Camera camera,
	float4x4 pose,
	ftl::cuda::ILWParams params,
    float rate) {

    const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	const float d0_new = d_new.tex2D((int)x,(int)y);
	const float d0_old = d_old.tex2D((int)x,(int)y);
	if (d0_new == 0.0f) return;  // No correspondence found
    
    if (x < d_old.width() && y < d_old.height()) {
		//const float4 world = p(x,y);
		//if (world.x == MINF) return;

		float delta = 0.0f; //make_float4(0.0f, 0.0f, 0.0f, 0.0f); //ev.tex2D((int)x,(int)y);
		float contrib = 0.0f;

		// Calculate screen space distortion with neighbours
		for (int v=-MOTION_RADIUS; v<=MOTION_RADIUS; ++v) {
			for (int u=-MOTION_RADIUS; u<=MOTION_RADIUS; ++u) {
				const float dn_new = d_new.tex2D((int)x+u,(int)y+v);
				const float dn_old = d_old.tex2D((int)x+u,(int)y+v);
				const float confn = conf.tex2D((int)x+u,(int)y+v);
				//const float3 pn = make_float3(p.tex2D((int)x+u,(int)y+v));
				//if (pn.x == MINF) continue;
				if (dn_new == 0.0f) continue;  // Neighbour has no new correspondence

				const float s = ftl::cuda::spatialWeighting(camera.screenToCam(x,y,d0_new), camera.screenToCam(x+u,y+v,dn_new), params.range); // ftl::cuda::weighting(fabs(d0_new - dn_new), params.range);
				contrib += (confn+0.01f) * s;
				delta += (confn+0.01f) * s * ((confn == 0.0f) ? dn_old : dn_new);
			}
		}

        if (contrib > 0.0f) {
            //const float3 newworld = pose * camera.screenToCam(x, y, vec0.x + rate * ((delta / contrib) - vec0.x));
			//p(x,y) = make_float4(newworld, world.w); //world + rate * (vec / contrib);
			
			d_old(x,y) = d0_old + rate * ((delta / contrib) - d0_old);
        }
    }
}


void ftl::cuda::move_points(
        ftl::cuda::TextureObject<float> &d_old,
		ftl::cuda::TextureObject<float> &d_new,
		ftl::cuda::TextureObject<float> &conf,
        const ftl::rgbd::Camera &camera,
		const float4x4 &pose,
		const ftl::cuda::ILWParams &params,
        float rate,
        int radius,
        cudaStream_t stream) {

    const dim3 gridSize((d_old.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (d_old.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    switch (radius) {
    case 9 : move_points_kernel<9><<<gridSize, blockSize, 0, stream>>>(d_old,d_new,conf,camera, pose, params, rate); break;
    case 5 : move_points_kernel<5><<<gridSize, blockSize, 0, stream>>>(d_old,d_new,conf,camera, pose, params, rate); break;
    case 3 : move_points_kernel<3><<<gridSize, blockSize, 0, stream>>>(d_old,d_new,conf,camera, pose, params, rate); break;
    case 1 : move_points_kernel<1><<<gridSize, blockSize, 0, stream>>>(d_old,d_new,conf,camera, pose, params, rate); break;
    case 0 : move_points_kernel<0><<<gridSize, blockSize, 0, stream>>>(d_old,d_new,conf,camera, pose, params, rate); break;
    }

    cudaSafeCall( cudaGetLastError() );
}
