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
	uchar4 c = colour.tex2D((int)x,(int)y);

	// Calculate discontinuity mask

	// Fill missing depths
	if (d < camera.minDepth || d > camera.maxDepth) {
		float depth_accum = 0.0f;
		float contrib = 0.0f;

		for (int v=-RADIUS; v<=RADIUS; ++v) {
			for (int u=-RADIUS; u<=RADIUS; ++u) {
				uchar4 c2 = colour.tex2D((int)x+u,(int)y+v);
				float d2 = depth_in.tex2D((int)x+u,(int)y+v);
				if (d2 >= camera.minDepth && d2 <= camera.maxDepth) {
					float w = ftl::cuda::colourWeighting(c, c2, params.colour_smooth);
					depth_accum += d2*w;
					contrib += w;
				}
			}
		}

		if (contrib >= 0.0f) d = depth_accum / contrib;
	}

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

	preprocess_kernel<3><<<gridSize, blockSize, 0, stream>>>(depth_in, depth_out, colour, mask, camera, params);

	cudaSafeCall( cudaGetLastError() );
}

//==============================================================================

template<int FUNCTION>
__device__ float costFunction(const ftl::cuda::ILWParams &params, float dweight, float cweight);

template <>
__device__ inline float costFunction<0>(const ftl::cuda::ILWParams &params, float dweight, float cweight) {
	return 1.0f - (params.cost_ratio * (cweight) + (1.0f - params.cost_ratio) * dweight);
}

template <>
__device__ inline float costFunction<1>(const ftl::cuda::ILWParams &param, float dweight, float cweight) {
	return 1.0f - (cweight * cweight * dweight);
}

template <>
__device__ inline float costFunction<2>(const ftl::cuda::ILWParams &param, float dweight, float cweight) {
	return 1.0f - (dweight * dweight * cweight);
}

template <>
__device__ inline float costFunction<3>(const ftl::cuda::ILWParams &params, float dweight, float cweight) {
	return (dweight == 0.0f) ? 1.0f : 1.0f - (params.cost_ratio * (cweight) + (1.0f - params.cost_ratio) * dweight);
}

template<int COR_STEPS, int FUNCTION> 
__global__ void correspondence_energy_vector_kernel(
        TextureObject<float> d1,
        TextureObject<float> d2,
        TextureObject<uchar4> c1,
        TextureObject<uchar4> c2,
        TextureObject<float> dout,
        TextureObject<float> conf,
        float4x4 pose1,
        float4x4 pose1_inv,
        float4x4 pose2,  // Inverse
        Camera cam1,
        Camera cam2, ftl::cuda::ILWParams params) {

    // Each warp picks point in p1
    //const int tid = (threadIdx.x + threadIdx.y * blockDim.x);
	const int x = (blockIdx.x*blockDim.x + threadIdx.x); // / WARP_SIZE;
    const int y = blockIdx.y*blockDim.y + threadIdx.y;
    
    //const float3 world1 = make_float3(p1.tex2D(x, y));
    const float depth1 = d1.tex2D(x,y); //(pose1_inv * world1).z;  // Initial starting depth
	if (depth1 < cam1.minDepth || depth1 > cam1.maxDepth) return;

	// TODO: Temporary hack to ensure depth1 is present
	//const float4 temp = vout.tex2D(x,y);
	//vout(x,y) =  make_float4(depth1, 0.0f, temp.z, temp.w);
	
	const float3 world1 = pose1 * cam1.screenToCam(x,y,depth1);

    const uchar4 colour1 = c1.tex2D(x, y);

    float bestcost = 1.1f;
    float avgcost = 0.0f;
    float bestdepth;
    int count = 0;
    
	const float step_interval = params.range / (COR_STEPS / 2);
	
	const float3 rayStep_world = pose1.getFloat3x3() * cam1.screenToCam(x,y,step_interval);
	const float3 rayStart_2 = pose2 * world1;
	const float3 rayStep_2 = pose2.getFloat3x3() * rayStep_world;

    // Project to p2 using cam2
    // Each thread takes a possible correspondence and calculates a weighting
    //const int lane = tid % WARP_SIZE;
	for (int i=0; i<COR_STEPS; ++i) {
		const int j = i - (COR_STEPS/2);
		const float depth_adjust = (float)j * step_interval + depth1;

        // Calculate adjusted depth 3D point in camera 2 space
        const float3 worldPos = world1 + j * rayStep_world; //(pose1 * cam1.screenToCam(x, y, depth_adjust));
        const float3 camPos = rayStart_2 + j * rayStep_2; //pose2 * worldPos;
        const uint2 screen = cam2.camToScreen<uint2>(camPos);

        if (screen.x >= cam2.width || screen.y >= cam2.height) continue;

		// Generate a depth correspondence value
		const float depth2 = d2.tex2D((int)screen.x, (int)screen.y);
		const float dweight = ftl::cuda::weighting(fabs(depth2 - camPos.z), params.spatial_smooth);
		
		// Generate a colour correspondence value
		const uchar4 colour2 = c2.tex2D((int)screen.x, (int)screen.y);
		const float cweight = ftl::cuda::colourWeighting(colour1, colour2, params.colour_smooth);

		const float cost = costFunction<FUNCTION>(params, dweight, cweight);

		// Cost is so bad, don't even consider this a valid option
		if (cost >= params.cost_threshold) continue;

		++count;
		avgcost += cost;
		if (cost < bestcost) {
			bestdepth = depth_adjust;
			bestcost = cost;
		}
    }

	//count = warpSum(count);
    const float mincost = bestcost; //warpMin(bestcost);
	//bool best = mincost == bestcost;
	avgcost /= count;
    const float confidence = (params.flags & ftl::cuda::kILWFlag_ColourConfidenceOnly) ? avgcost : (avgcost - mincost);

    if (mincost < 1.0f) {
        float old = conf.tex2D(x,y);

        if ((1.0f - mincost) * confidence > old) {
			dout(x,y) = bestdepth;
			conf(x,y) = (1.0f - mincost) * confidence;
		}
    }
}

void ftl::cuda::correspondence(
        TextureObject<float> &d1,
        TextureObject<float> &d2,
        TextureObject<uchar4> &c1,
        TextureObject<uchar4> &c2,
        TextureObject<float> &dout,
        TextureObject<float> &conf,
        float4x4 &pose1,
        float4x4 &pose1_inv,
        float4x4 &pose2,
        const Camera &cam1,
        const Camera &cam2, const ILWParams &params, int func,
        cudaStream_t stream) {

	const dim3 gridSize((d1.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (d1.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    //printf("COR SIZE %d,%d\n", p1.width(), p1.height());

	switch (func) {
    case 0: correspondence_energy_vector_kernel<16,0><<<gridSize, blockSize, 0, stream>>>(d1, d2, c1, c2, dout, conf, pose1, pose1_inv, pose2, cam1, cam2, params);
	case 1: correspondence_energy_vector_kernel<16,1><<<gridSize, blockSize, 0, stream>>>(d1, d2, c1, c2, dout, conf, pose1, pose1_inv, pose2, cam1, cam2, params);
	case 2: correspondence_energy_vector_kernel<16,2><<<gridSize, blockSize, 0, stream>>>(d1, d2, c1, c2, dout, conf, pose1, pose1_inv, pose2, cam1, cam2, params);
	case 3: correspondence_energy_vector_kernel<16,3><<<gridSize, blockSize, 0, stream>>>(d1, d2, c1, c2, dout, conf, pose1, pose1_inv, pose2, cam1, cam2, params);
	}

    cudaSafeCall( cudaGetLastError() );
}

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

				const float s = ftl::cuda::weighting(fabs(d0_new - dn_new), params.range);
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
