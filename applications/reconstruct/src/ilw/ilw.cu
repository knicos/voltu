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

//#define COR_WIN_RADIUS 17
//#define COR_WIN_SIZE (COR_WIN_RADIUS * COR_WIN_RADIUS)

template<int COR_STEPS> 
__global__ void correspondence_energy_vector_kernel(
        TextureObject<float4> p1,
        TextureObject<float4> p2,
        TextureObject<uchar4> c1,
        TextureObject<uchar4> c2,
        TextureObject<float4> vout,
        TextureObject<float> eout,
        float4x4 pose1,
        float4x4 pose1_inv,
        float4x4 pose2,  // Inverse
        Camera cam1,
        Camera cam2, ftl::cuda::ILWParams params) {

    // Each warp picks point in p1
    const int tid = (threadIdx.x + threadIdx.y * blockDim.x);
	const int x = (blockIdx.x*blockDim.x + threadIdx.x) / WARP_SIZE;
    const int y = blockIdx.y*blockDim.y + threadIdx.y;
    
    const float3 world1 = make_float3(p1.tex2D(x, y));
    const float depth1 = (pose1_inv * world1).z;  // Initial starting depth
    if (depth1 < cam1.minDepth || depth1 > cam1.maxDepth) return;

    const uchar4 colour1 = c1.tex2D(x, y);

    float bestcost = 1.1f;
    float avgcost = 0.0f;
    float bestdepth;
    int count = 0;
    
    const float step_interval = 0.05f / COR_STEPS;

    // Project to p2 using cam2
    // Each thread takes a possible correspondence and calculates a weighting
    const int lane = tid % WARP_SIZE;
	for (int i=lane; i<COR_STEPS; i+=WARP_SIZE) {
        const float depth_adjust = (float)(i - (COR_STEPS / 2)) * step_interval + depth1;

        // Calculate adjusted depth 3D point in camera 2 space
        const float3 worldPos = (pose1 * cam1.screenToCam(x, y, depth_adjust));
        const float3 camPos = pose2 * worldPos;
        const uint2 screen = cam2.camToScreen<uint2>(camPos);

        if (screen.x >= cam2.width || screen.y >= cam2.height) continue;

        // Now do correspondence evaluation at "screen" location in camera 2
        const float3 world2 = make_float3(p2.tex2D((int)screen.x, (int)screen.y));
        if ((params.flags & ftl::cuda::kILWFlag_IgnoreBad) && world2.x == MINF) continue;
        const uchar4 colour2 = c2.tex2D((int)screen.x, (int)screen.y);

        // Determine degree of correspondence
		float cost = 1.0f - ftl::cuda::spatialWeighting(world1, world2, params.spatial_smooth);
		// Point is too far away to even count
		if (world2.x != MINF && cost == 1.0f) continue;

        // Mix ratio of colour and distance costs
        const float ccost = 1.0f - ftl::cuda::colourWeighting(colour1, colour2, params.colour_smooth);
        if ((params.flags & ftl::cuda::kILWFlag_SkipBadColour) && ccost == 1.0f) continue;
        cost = params.cost_ratio * (ccost) + (1.0f - params.cost_ratio) * cost;
        //cost /= 2.0f;

		++count;
		avgcost += cost;
        if (world2.x != MINF && cost < bestcost) {
            bestdepth = depth_adjust;
            bestcost = cost;
        }
    }

	count = warpSum(count);
    const float mincost = warpMin(bestcost);
	bool best = mincost == bestcost;
	avgcost = warpSum(avgcost) / count;
    const float confidence = (avgcost - mincost);

    // FIXME: Multiple threads in warp could match this.
    if (best && mincost < 1.0f) {
        float3 tvecA = pose1 * cam1.screenToCam(x, y, bestdepth);
        //float3 tvecB = pose1 * world1;
        //if (params.flags & ftl::cuda::kILWFlag_RestrictZ) {
        //    tvecA.x = tvecB.x;
        //    tvecA.y = tvecB.y;
        //}
        tvecA = tvecA - world1;
        vout(x,y) =  make_float4(
            tvecA.x, // * (1.0f - mincost) * confidence,
            tvecA.y, // * (1.0f - mincost) * confidence,
            tvecA.z, // * (1.0f - mincost) * confidence,
            (1.0f - mincost) * confidence);
			
		//eout(x,y) = max(eout(x,y), (length(bestpoint-world1) / 0.04f) * 7.0f);
		//eout(x,y) = max(eout(x,y), (1.0f - mincost) * 7.0f);
		//eout(x,y) = max(eout(x, y), (1.0f - mincost) * confidence * (length(bestpoint-world1) / 0.04f) * 12.0f);
		eout(x,y) = max(eout(x, y), (1.0f - mincost) * confidence * 12.0f);
		//eout(x,y) = max(eout(x, y), confidence * 12.0f);
    } else if (mincost >= 1.0f && lane == 0) {
        //vout(x,y) = make_float4(0.0f);
        //eout(x,y) = 0.0f;
    }
}

void ftl::cuda::correspondence_energy_vector(
        TextureObject<float4> &p1,
        TextureObject<float4> &p2,
        TextureObject<uchar4> &c1,
        TextureObject<uchar4> &c2,
        TextureObject<float4> &vout,
        TextureObject<float> &eout,
        float4x4 &pose1,
        float4x4 &pose1_inv,
        float4x4 &pose2,
        const Camera &cam1,
        const Camera &cam2, const ILWParams &params, int win,
        cudaStream_t stream) {

    const dim3 gridSize((p1.width() + 2 - 1)/2, (p1.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(2*WARP_SIZE, T_PER_BLOCK);

    //printf("COR SIZE %d,%d\n", p1.width(), p1.height());

    correspondence_energy_vector_kernel<64><<<gridSize, blockSize, 0, stream>>>(p1, p2, c1, c2, vout, eout, pose1, pose1_inv, pose2, cam1, cam2, params);

    //switch (win) {
    //case 17     : correspondence_energy_vector_kernel<17><<<gridSize, blockSize, 0, stream>>>(p1, p2, c1, c2, vout, eout, pose1, pose1_inv, pose2, cam1, cam2, params); break;
    //case 9      : correspondence_energy_vector_kernel<9><<<gridSize, blockSize, 0, stream>>>(p1, p2, c1, c2, vout, eout, pose1, pose1_inv, pose2, cam1, cam2, params); break;
    //case 5      : correspondence_energy_vector_kernel<5><<<gridSize, blockSize, 0, stream>>>(p1, p2, c1, c2, vout, eout, pose1, pose1_inv, pose2, cam1, cam2, params); break;
    //}
    cudaSafeCall( cudaGetLastError() );
}

//==============================================================================

//#define MOTION_RADIUS 9

template <int MOTION_RADIUS>
__global__ void move_points_kernel(
    ftl::cuda::TextureObject<float4> p,
    ftl::cuda::TextureObject<float4> ev,
    ftl::rgbd::Camera camera,
    float rate) {

    const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;
    
    if (x < p.width() && y < p.height()) {
		const float4 world = p(x,y);
		if (world.x == MINF) return;

		float4 vec = make_float4(0.0f, 0.0f, 0.0f, 0.0f); //ev.tex2D((int)x,(int)y);
		float contrib = 0.0f;

		// Calculate screen space distortion with neighbours
		for (int v=-MOTION_RADIUS; v<=MOTION_RADIUS; ++v) {
			for (int u=-MOTION_RADIUS; u<=MOTION_RADIUS; ++u) {
				const float4 vecn = ev.tex2D((int)x+u,(int)y+v);
				const float3 pn = make_float3(p.tex2D((int)x+u,(int)y+v));
				if (pn.x == MINF) continue;

				const float s = ftl::cuda::spatialWeighting(pn, make_float3(world), 0.01f);
				contrib += vecn.w * s;
				vec += vecn.w * s * vecn;
			}
		}

        if (vec.w > 0.0f) {
            p(x,y) = world + rate * (vec / contrib);
        }
    }
}


void ftl::cuda::move_points(
        ftl::cuda::TextureObject<float4> &p,
        ftl::cuda::TextureObject<float4> &v,
        const ftl::rgbd::Camera &camera,
        float rate,
        int radius,
        cudaStream_t stream) {

    const dim3 gridSize((p.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (p.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    switch (radius) {
    case 9 : move_points_kernel<9><<<gridSize, blockSize, 0, stream>>>(p,v,camera,rate); break;
    case 5 : move_points_kernel<5><<<gridSize, blockSize, 0, stream>>>(p,v,camera,rate); break;
    case 3 : move_points_kernel<3><<<gridSize, blockSize, 0, stream>>>(p,v,camera,rate); break;
    case 1 : move_points_kernel<1><<<gridSize, blockSize, 0, stream>>>(p,v,camera,rate); break;
    case 0 : move_points_kernel<0><<<gridSize, blockSize, 0, stream>>>(p,v,camera,rate); break;
    }

    cudaSafeCall( cudaGetLastError() );
}
