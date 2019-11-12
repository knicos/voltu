#include "ilw_cuda.hpp"
#include <ftl/cuda/weighting.hpp>

using ftl::cuda::TextureObject;
using ftl::rgbd::Camera;
using ftl::cuda::Mask;

#define T_PER_BLOCK 8

template<int FUNCTION>
__device__ float weightFunction(const ftl::cuda::ILWParams &params, float dweight, float cweight);

template <>
__device__ inline float weightFunction<0>(const ftl::cuda::ILWParams &params, float dweight, float cweight) {
	return (params.cost_ratio * (cweight) + (1.0f - params.cost_ratio) * dweight);
}

template <>
__device__ inline float weightFunction<1>(const ftl::cuda::ILWParams &param, float dweight, float cweight) {
	return (cweight * cweight * dweight);
}

template <>
__device__ inline float weightFunction<2>(const ftl::cuda::ILWParams &param, float dweight, float cweight) {
	return (dweight * dweight * cweight);
}

template <>
__device__ inline float weightFunction<3>(const ftl::cuda::ILWParams &params, float dweight, float cweight) {
	return (dweight == 0.0f) ? 0.0f : (params.cost_ratio * (cweight) + (1.0f - params.cost_ratio) * dweight);
}

template <>
__device__ inline float weightFunction<4>(const ftl::cuda::ILWParams &params, float dweight, float cweight) {
	return cweight;
}

template<int COR_STEPS, int FUNCTION> 
__global__ void correspondence_energy_vector_kernel(
        TextureObject<float> d1,
        TextureObject<float> d2,
        TextureObject<uchar4> c1,
        TextureObject<uchar4> c2,
        TextureObject<float> dout,
		TextureObject<float> conf,
		TextureObject<int> mask,
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

    const auto colour1 = c1.tex2D((float)x+0.5f, (float)y+0.5f);

	float bestdepth = 0.0f;
	float bestweight = 0.0f;
	float bestcolour = 0.0f;
	float bestdweight = 0.0f;
	float totalcolour = 0.0f;
	int count = 0;
	float contrib = 0.0f;
    
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
        const float2 screen = cam2.camToScreen<float2>(camPos);

        if (screen.x >= cam2.width || screen.y >= cam2.height) continue;

		// Generate a depth correspondence value
		const float depth2 = d2.tex2D(int(screen.x+0.5f), int(screen.y+0.5f));
		const float dweight = ftl::cuda::weighting(fabs(depth2 - camPos.z), params.spatial_smooth);
		//const float dweight = ftl::cuda::weighting(fabs(depth_adjust - depth1), 2.0f*params.range);
		
		// Generate a colour correspondence value
		const auto colour2 = c2.tex2D(screen.x, screen.y);
		const float cweight = ftl::cuda::colourWeighting(colour1, colour2, params.colour_smooth);

		const float weight = weightFunction<FUNCTION>(params, dweight, cweight);

		++count;
		contrib += weight;
		bestcolour = max(cweight, bestcolour);
		bestdweight = max(dweight, bestdweight);
		totalcolour += cweight;
		if (weight > bestweight) {
			bestweight = weight;
			bestdepth = depth_adjust;
		}
    }

	const float avgcolour = totalcolour/(float)count;
	const float confidence = bestcolour / totalcolour; //bestcolour - avgcolour;
	
	Mask m(mask.tex2D(x,y));

    //if (bestweight > 0.0f) {
        float old = conf.tex2D(x,y);

        if (bestweight * confidence > old) {
			dout(x,y) = bestdepth;
			conf(x,y) = bestweight * confidence;
		}
	//}
	
	// If a good enough match is found, mark dodgy depth as solid
	if ((m.isFilled() || m.isDiscontinuity()) && (bestweight > params.match_threshold)) mask(x,y) = 0;
}

void ftl::cuda::correspondence(
        TextureObject<float> &d1,
        TextureObject<float> &d2,
        TextureObject<uchar4> &c1,
        TextureObject<uchar4> &c2,
        TextureObject<float> &dout,
		TextureObject<float> &conf,
		TextureObject<int> &mask,
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
    case 0: correspondence_energy_vector_kernel<16,0><<<gridSize, blockSize, 0, stream>>>(d1, d2, c1, c2, dout, conf, mask, pose1, pose1_inv, pose2, cam1, cam2, params); break;
	case 1: correspondence_energy_vector_kernel<16,1><<<gridSize, blockSize, 0, stream>>>(d1, d2, c1, c2, dout, conf, mask, pose1, pose1_inv, pose2, cam1, cam2, params); break;
	case 2: correspondence_energy_vector_kernel<16,2><<<gridSize, blockSize, 0, stream>>>(d1, d2, c1, c2, dout, conf, mask, pose1, pose1_inv, pose2, cam1, cam2, params); break;
	case 3: correspondence_energy_vector_kernel<16,3><<<gridSize, blockSize, 0, stream>>>(d1, d2, c1, c2, dout, conf, mask, pose1, pose1_inv, pose2, cam1, cam2, params); break;
	case 4: correspondence_energy_vector_kernel<16,4><<<gridSize, blockSize, 0, stream>>>(d1, d2, c1, c2, dout, conf, mask, pose1, pose1_inv, pose2, cam1, cam2, params); break;
	}

    cudaSafeCall( cudaGetLastError() );
}

//==============================================================================

__global__ void mask_filter_kernel(
		TextureObject<float> depth,
		TextureObject<int> mask) {
	
	const int x = (blockIdx.x*blockDim.x + threadIdx.x);
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x >= 0 && x < depth.width() && y >=0 && y < depth.height()) {
		Mask m(mask.tex2D(x,y));
		if (m.isFilled()) {
			depth(x,y) = MINF;
		}
	}
}


void ftl::cuda::mask_filter(
		TextureObject<float> &depth,
		TextureObject<int> &mask,
		cudaStream_t stream) {
	const dim3 gridSize((depth.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	mask_filter_kernel<<<gridSize, blockSize, 0, stream>>>(
		depth, mask
	);
	cudaSafeCall( cudaGetLastError() );
}
