#include "mvmls_cuda.hpp"
#include <ftl/cuda/weighting.hpp>
#include <ftl/operators/cuda/mask.hpp>
#include <ftl/cuda/warp.hpp>

using ftl::cuda::TextureObject;
using ftl::rgbd::Camera;
using ftl::cuda::Mask;
using ftl::cuda::MvMLSParams;

#define T_PER_BLOCK 8
#define WARP_SIZE 32

#include "correspondence_common.hpp"

template<int COR_STEPS> 
__global__ void corresponding_depth_kernel(
        TextureObject<float> d1,
        TextureObject<float> d2,
        TextureObject<short2> screenOut,
		TextureObject<float> conf,
		TextureObject<uint8_t> mask,
        float4x4 pose,
        Camera cam1,
        Camera cam2, ftl::cuda::MvMLSParams params) {

	const int2 pt = block4x4<2>();


	// FIXME: Don't return like this due to warp operations
    if (pt.x >= 0 && pt.y >=0 && pt.x < screenOut.width() && pt.y < screenOut.height()) {
		screenOut(pt) = make_short2(-1,-1);
		conf(pt) = 0.0f;
    
		const float depth1 = d1.tex2D(pt);
		// TODO: If all depths within a half warp here are bad then can return
		// early.
		//if (__ballot_sync(FULL_MASK, depth1 > cam1.minDepth && depth1 < cam1.maxDepth) & ((threadIdx.x/16 == 1) ? 0xFFFF0000 : 0xFFFF) == 0) return;

        short2 bestScreen = make_short2(-1,-1);
        float bestcost = PINF;
		
		const float3 camPosOrigin = pose * cam1.screenToCam(pt.x,pt.y,depth1);
        const float2 lineOrigin = cam2.camToScreen<float2>(camPosOrigin);
        const float3 camPosDistant = pose * cam1.screenToCam(pt.x,pt.y,depth1 + 0.4f);
        const float2 lineDistant = cam2.camToScreen<float2>(camPosDistant);
        const float lineM = (lineDistant.y - lineOrigin.y) / (lineDistant.x - lineOrigin.x);
		const float depthM = 0.4f / (lineDistant.x - lineOrigin.x);
		const float depthM2 = (camPosDistant.z - camPosOrigin.z) / (lineDistant.x - lineOrigin.x);
        float2 linePos;
        linePos.x = lineOrigin.x - float((COR_STEPS/2));
        linePos.y = lineOrigin.y - (float((COR_STEPS/2)) * lineM);
		float depthPos2 = camPosOrigin.z - (float((COR_STEPS/2)) * depthM2);

		// Pre-calculate a depth resolution coeficient
		const float depthCoef = (1.0f / (cam1.baseline *cam1.fx)) * params.spatial_smooth;
        
        uint badMask = 0;
        int bestStep = COR_STEPS/2;
		//float bestSource = 0.0f;

		// Scan from -COR_STEPS/2 to +COR_STEPS/2, where COR_STEPS is the number
		// of pixels to check in the second camera for the current first camera
		// pixel.
		// TODO: Could consider sub pixel accuracy using linear interpolation
		// between the depths of subsequent pixels to find a better minimum.
		// This depends on GPU resource limitations for the calculation.

		#pragma unroll
        for (int i=0; i<COR_STEPS; ++i) {			
			// Read the actual depth at current correspondence check point
            const float depth2 = d2.tex2D(int(linePos.x+0.5f), int(linePos.y+0.5f));
            
			// Record which correspondences are invalid
            badMask |= (isBadCor(depth2, linePos, cam2)) ? 1 << i : 0;
	
			// Sum of square of normalised depth error from both cameras
			// Normalised using depth resolution in respective camera, this
			// should bias in favour of moving the lower res camera point
			// Effectively this is a squared distance from ideal location.
			float cost = square(min(1.0f, fabs(depth2-depthPos2) / (depthCoef * depth2 * depth2)));
			cost += square(min(1.0f, (fabs(float(i-COR_STEPS/2))*depthM) / (depthCoef * depth1 * depth1)));
			
			// TODO: Perhaps allow per pixel variations but with additional cost
			// the greater they are from the warp concensus best cost

			// Sum all squared distance errors in a 4x4 pixel neighborhood
			float wcost = ftl::cuda::halfWarpSum(cost) / 16.0f;
			cost = (wcost < 1.0f) ? (wcost + cost) / 2.0f : 1.0f;  // Enables per pixel adjustments

			// Record the best result
            bestStep = (cost < bestcost) ? i : bestStep;
			bestScreen = (cost < bestcost) ? make_short2(linePos.x+0.5f,linePos.y+0.5f) : bestScreen;
			bestcost = min(bestcost, cost);
                
			
			// Move to next correspondence check point
			depthPos2 += depthM2;
            linePos.x += 1.0f;
            linePos.y += lineM;
        }

		float bestadjust = float(bestStep-(COR_STEPS/2))*depthM;
		
		//if (bestStep == 0 ||bestStep == COR_STEPS-1) {
		//	printf("Bad step: %f\n", bestcost);
		//}

        // Detect matches to boundaries, and discard those
        uint stepMask = 1 << bestStep;
		if ((stepMask & badMask) || (stepMask & (badMask << 1)) || (stepMask & (badMask >> 1))) bestcost = PINF;

		// Check that no boundaries were matched in 4x4 block
		bestcost = ftl::cuda::halfWarpMax(bestcost);
		
		// If a match was found then record it
		if (depth1 > cam1.minDepth && depth1 < cam1.maxDepth && bestcost < 1.0f) {
			// Delay making the depth change until later.
			conf(pt) = bestadjust;
			auto m = mask(pt);
			m &= ~Mask::kMask_Bad;
			mask(pt) = m | Mask::kMask_Correspondence;
			screenOut(pt) = bestScreen;
		}

		if (depth1 > cam1.minDepth && depth1 < cam1.maxDepth && bestcost > 2.0f) {
			auto m = mask(pt);
			mask(pt) = (m & Mask::kMask_Correspondence) ? m : m | Mask::kMask_Bad;
		}
    }
}

void ftl::cuda::correspondence(
		TextureObject<float> &d1,
		TextureObject<float> &d2,
		TextureObject<short2> &screen,
		TextureObject<float> &conf,
		TextureObject<uint8_t> &mask,
		float4x4 &pose2,
		const Camera &cam1,
		const Camera &cam2, const MvMLSParams &params, int func,
		cudaStream_t stream) {

	// FIXME: Check the grid size makes sense
	//const dim3 gridSize((d1.width() + 1), (d1.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	//const dim3 blockSize(WARP_SIZE, 2);

	const dim3 gridSize((d1.width() + 8 - 1)/8, (d1.height() + 8 - 1)/8);
	const dim3 blockSize(WARP_SIZE, 2);

	switch (func) {
	case 32: corresponding_depth_kernel<32><<<gridSize, blockSize, 0, stream>>>(d1, d2, screen, conf, mask, pose2, cam1, cam2, params); break;
	case 16: corresponding_depth_kernel<16><<<gridSize, blockSize, 0, stream>>>(d1, d2, screen, conf, mask, pose2, cam1, cam2, params); break;
	case 8: corresponding_depth_kernel<8><<<gridSize, blockSize, 0, stream>>>(d1, d2, screen, conf, mask, pose2, cam1, cam2, params); break;
	case 4: corresponding_depth_kernel<4><<<gridSize, blockSize, 0, stream>>>(d1, d2, screen, conf, mask, pose2, cam1, cam2, params); break;
	case 2: corresponding_depth_kernel<2><<<gridSize, blockSize, 0, stream>>>(d1, d2, screen, conf, mask, pose2, cam1, cam2, params); break;
	}

	cudaSafeCall( cudaGetLastError() );
}
