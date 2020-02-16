#include "mvmls_cuda.hpp"
#include <ftl/cuda/weighting.hpp>
#include <ftl/operators/mask_cuda.hpp>
#include <ftl/cuda/warp.hpp>

using ftl::cuda::TextureObject;
using ftl::rgbd::Camera;
using ftl::cuda::Mask;
using ftl::cuda::MvMLSParams;

#define T_PER_BLOCK 8
#define WARP_SIZE 32
#define INTERVAL 1

#include "correspondence_common.hpp"

template<int FUNCTION>
__device__ float weightFunction(const ftl::cuda::MvMLSParams &params, float dweight, float cweight);

/*template <>
__device__ inline float weightFunction<0>(const ftl::cuda::MvMLSParams &params, float dweight, float cweight) {
	return (params.cost_ratio * (cweight) + (1.0f - params.cost_ratio) * dweight);
}

template <>
__device__ inline float weightFunction<1>(const ftl::cuda::MvMLSParams &param, float dweight, float cweight) {
	return (cweight * cweight * dweight);
}

template <>
__device__ inline float weightFunction<2>(const ftl::cuda::MvMLSParams &param, float dweight, float cweight) {
	return (dweight * dweight * cweight);
}

template <>
__device__ inline float weightFunction<3>(const ftl::cuda::MvMLSParams &params, float dweight, float cweight) {
	return (dweight == 0.0f) ? 0.0f : (params.cost_ratio * (cweight) + (1.0f - params.cost_ratio) * dweight);
}

template <>
__device__ inline float weightFunction<4>(const ftl::cuda::MvMLSParams &params, float dweight, float cweight) {
	return cweight;
}

template <>
__device__ inline float weightFunction<5>(const ftl::cuda::MvMLSParams &params, float dweight, float cweight) {
	return (cweight > 0.0f) ? dweight : 0.0f;
}*/

__device__ inline float distance(float4 p1, float4 p2) {
	return min(1.0f, max(max(fabsf(p1.x - p2.x),fabsf(p1.y - p2.y)), fabsf(p1.z - p2.z))/ 10.0f);
	//return min(1.0f, ftl::cuda::colourDistance(p1, p2) / 10.0f);
}

__device__ inline int halfWarpCensus(float e) {
	float e0 = __shfl_sync(FULL_MASK, e, (threadIdx.x >= 16) ? 16 : 0, WARP_SIZE);
	int c = (e > e0) ? 1 << (threadIdx.x % 16) : 0;
	for (int i = WARP_SIZE/4; i > 0; i /= 2) {
		const int other = __shfl_xor_sync(FULL_MASK, c, i, WARP_SIZE);
		c |= other;
	}
	return c;
}

__device__ inline float halfWarpBest(float e, float c) {
	for (int i = WARP_SIZE/4; i > 0; i /= 2) {
		const float o1 = __shfl_xor_sync(FULL_MASK, e, i, WARP_SIZE);
		const float o2 = __shfl_xor_sync(FULL_MASK, c, i, WARP_SIZE);
		e = (o2 > c) ? o1 : e;
	}
	return e;
}

__device__ inline float4 relativeDelta(const float4 &e) {
	const float e0x = __shfl_sync(FULL_MASK, e.x, 0, WARP_SIZE/2);
	const float e0y = __shfl_sync(FULL_MASK, e.y, 0, WARP_SIZE/2);
	const float e0z = __shfl_sync(FULL_MASK, e.z, 0, WARP_SIZE/2);
	return make_float4(e.x-e0x, e.y-e0y, e.z-e0z, 0.0f);
}

/**
 * See: Birchfield S. et al. (1998). A pixel dissimilarity measure that is
 * insensitive to image sampling. IEEE Transactions on Pattern Analysis and
 * Machine Intelligence.
 */
__device__ float dissimilarity(const float4 &l, const float4 &rp, const float4 &rc, const float4 &rn) {
	const float rpd = distance((rc - rp) * 0.5f + rp, l);
	const float rnd = distance((rc - rn) * 0.5f + rn, l);
	const float rcd = distance(rc, l);
	return min(min(rpd, rnd), rcd);
}



template<int COR_STEPS, int FUNCTION> 
__global__ void corresponding_point_kernel(
        TextureObject<float> d1,
        TextureObject<float> d2,
        TextureObject<uchar4> c1,
        TextureObject<uchar4> c2,
        TextureObject<short2> screenOut,
		TextureObject<float> conf,
		TextureObject<uint8_t> mask,
        float4x4 pose,
        Camera cam1,
        Camera cam2, ftl::cuda::MvMLSParams params) {
	
	//const int tid = (threadIdx.x + threadIdx.y * blockDim.x);
	const int x = (blockIdx.x*8 + (threadIdx.x%4) + 4*(threadIdx.x / 16)); // / WARP_SIZE;
	const int y = blockIdx.y*8 + threadIdx.x/4 + 4*threadIdx.y;


    if (x >= 0 && y >=0 && x < screenOut.width() && y < screenOut.height()) {
        screenOut(x,y) = make_short2(-1,-1);
    
        //const float3 world1 = make_float3(p1.tex2D(x, y));
        const float depth1 = d1.tex2D(x,y); //(pose1_inv * world1).z;  // Initial starting depth
        if (depth1 < cam1.minDepth || depth1 > cam1.maxDepth) return;

        // TODO: Temporary hack to ensure depth1 is present
        //const float4 temp = vout.tex2D(x,y);
        //vout(x,y) =  make_float4(depth1, 0.0f, temp.z, temp.w);
        
        //const float3 world1 = pose1 * cam1.screenToCam(x,y,depth1);

        const auto colour1 = c1.tex2D((float)x+0.5f, (float)y+0.5f);

        //float bestdepth = 0.0f;
        short2 bestScreen = make_short2(-1,-1);
		//float bestdepth = 0.0f;
		//float bestdepth2 = 0.0f;
        float bestweight = 0.0f;
        float bestcolour = 0.0f;
        //float bestdweight = 0.0f;
        float totalcolour = 0.0f;
        //int count = 0;
        //float contrib = 0.0f;
		
		const float3 camPosOrigin = pose * cam1.screenToCam(x,y,depth1);
        const float2 lineOrigin = cam2.camToScreen<float2>(camPosOrigin);
        const float3 camPosDistant = pose * cam1.screenToCam(x,y,depth1 + 10.0f);
        const float2 lineDistant = cam2.camToScreen<float2>(camPosDistant);
        const float lineM = (lineDistant.y - lineOrigin.y) / (lineDistant.x - lineOrigin.x);
		const float depthM = 10.0f / (lineDistant.x - lineOrigin.x);
		const float depthM2 = (camPosDistant.z - camPosOrigin.z) / (lineDistant.x - lineOrigin.x);
        float2 linePos;
        linePos.x = lineOrigin.x - ((COR_STEPS/2));
        linePos.y = lineOrigin.y - (((COR_STEPS/2)) * lineM);
		//float depthPos = depth1 - (float((COR_STEPS/2)) * depthM);
		float depthPos2 = camPosOrigin.z - (float((COR_STEPS/2)) * depthM2);
		//const float depthCoef = cam1.baseline*cam1.fx;

		const float depthCoef = (1.0f / cam1.fx) * 10.0f;
        
        uint badMask = 0;
        int bestStep = COR_STEPS/2;


        // Project to p2 using cam2
        // Each thread takes a possible correspondence and calculates a weighting
        //const int lane = tid % WARP_SIZE;
        for (int i=0; i<COR_STEPS; ++i) {			
			//float weight = 1.0f; //(linePos.x >= cam2.width || linePos.y >= cam2.height) ? 0.0f : 1.0f;

			// Generate a colour correspondence value
            const auto colour2 = c2.tex2D(linePos.x, linePos.y);

            // TODO: Check if other colour dissimilarities are better...
            const float cweight = ftl::cuda::colourWeighting(colour1, colour2, params.colour_smooth);

            // Generate a depth correspondence value
            const float depth2 = d2.tex2D(int(linePos.x+0.5f), int(linePos.y+0.5f));
            
            // Record which correspondences are invalid
            badMask |= (
					depth2 <= cam2.minDepth ||
					depth2 >= cam2.maxDepth ||
					linePos.x < 0.5f ||
					linePos.y < 0.5f ||
					linePos.x >= d2.width()-0.5f ||
					linePos.y >= d2.height()-0.5f
				) ? 1 << i : 0;
			
			//if (FUNCTION == 1) {
			// TODO: Spatial smooth must be disparity discon threshold of largest depth in original camera space.
			// ie. if depth1 > depth2 then depth1 has the largest error potential and the resolution at that
			// depth is used to determine the spatial smoothing amount.
	
			const float maxdepth = max(depth1, depth2);
			const float smooth = depthCoef * maxdepth;
			float weight = ftl::cuda::weighting(fabs(depth2 - depthPos2), cweight*smooth);
			//weight = ftl::cuda::halfWarpSum(weight);
			//} else {
			//	const float dweight = ftl::cuda::weighting(fabs(depth2 - depthPos2), params.spatial_smooth);
            //	weight *= weightFunction<FUNCTION>(params, dweight, cweight);
			//}
            //const float dweight = ftl::cuda::weighting(fabs(depth_adjust), 10.0f*params.range);

            //weight *= weightFunction<FUNCTION>(params, dweight, cweight);

            //++count;

            bestcolour = max(cweight, bestcolour);
            totalcolour += cweight;

            //bestdepth = (weight > bestweight) ? depthPos : bestdepth;
            bestStep = (weight > bestweight) ? i : bestStep;
			
			bestweight = max(bestweight, weight);
                
			
			//depthPos += depthM;
			depthPos2 += depthM2;
            linePos.x += 1.0f;
            linePos.y += lineM;
        }

        //const float avgcolour = totalcolour/(float)count;
        const float confidence = ((bestcolour / totalcolour) - (1.0f / 16.0f)) * (1.0f + (1.0f/16.0f));
        float bestadjust = float(bestStep-(COR_STEPS/2))*depthM;

        // Detect matches to boundaries, and discard those
        uint stepMask = 1 << bestStep;
		if ((stepMask & badMask) || (stepMask & (badMask << 1)) || (stepMask & (badMask >> 1))) bestweight = 0.0f;
		
		//bestadjust = halfWarpBest(bestadjust, (bestweight > 0.0f) ? confidence : 0.0f);

        //Mask m(mask.tex2D(x,y));

        //if (bestweight > 0.0f) {
            float old = conf.tex2D(x,y);

            if (bestweight > 0.0f) {
				d1(x,y) = (0.4f*bestadjust) + depth1;
				//d2(bestScreen.x, bestScreen.y) = bestdepth2;
                //screenOut(x,y) = bestScreen;
				conf(x,y) = max(old,confidence); //bestweight * confidence;
				//conf(x,y) = max(old,fabs(bestadjust));
            }
        //}
        
        // If a good enough match is found, mark dodgy depth as solid
        //if ((m.isFilled() || m.isDiscontinuity()) && (bestweight > params.match_threshold)) mask(x,y) = 0;
    }
}

void ftl::cuda::correspondence(
        TextureObject<float> &d1,
        TextureObject<float> &d2,
        TextureObject<uchar4> &c1,
        TextureObject<uchar4> &c2,
        TextureObject<short2> &screen,
		TextureObject<float> &conf,
		TextureObject<uint8_t> &mask,
        float4x4 &pose2,
        const Camera &cam1,
        const Camera &cam2, const MvMLSParams &params, int func,
        cudaStream_t stream) {

	//const dim3 gridSize((d1.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (d1.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	//const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	const dim3 gridSize((d1.width() + 1), (d1.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(WARP_SIZE, 2);

    
    switch (func) {
    case 32: corresponding_point_kernel<32,1><<<gridSize, blockSize, 0, stream>>>(d1, d2, c1, c2, screen, conf, mask, pose2, cam1, cam2, params); break;
    case 16: corresponding_point_kernel<16,1><<<gridSize, blockSize, 0, stream>>>(d1, d2, c1, c2, screen, conf, mask, pose2, cam1, cam2, params); break;
    case 8: corresponding_point_kernel<8,1><<<gridSize, blockSize, 0, stream>>>(d1, d2, c1, c2, screen, conf, mask, pose2, cam1, cam2, params); break;
    case 4: corresponding_point_kernel<4,1><<<gridSize, blockSize, 0, stream>>>(d1, d2, c1, c2, screen, conf, mask, pose2, cam1, cam2, params); break;
    case 2: corresponding_point_kernel<2,1><<<gridSize, blockSize, 0, stream>>>(d1, d2, c1, c2, screen, conf, mask, pose2, cam1, cam2, params); break;
    }

    cudaSafeCall( cudaGetLastError() );
}
