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
	return min(1.0f, max(max(fabsf(p1.x - p2.x),fabsf(p1.y - p2.y)), fabsf(p1.z - p2.z))/ 255.0f);
	//return min(1.0f, ftl::cuda::colourDistance(p1, p2) / 10.0f);
}

__device__ inline int halfWarpCensus(float e) {
	float e0 = __shfl_sync(FULL_MASK, e, (threadIdx.x >= 16) ? 16+6 : 0+6, WARP_SIZE);
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

/**
 * One thread from each half warp writes a cost value to the correct cost
 * memory location.
 */
__device__ inline void writeCost(TextureObject<half> &costs, int x, int y, int i, float cost) {
	if ((threadIdx.x & 0xF) == 0) costs(x>>2, y>>2 + (costs.height()/16)*i) = __float2half(cost);
}

__device__ inline float readCost(TextureObject<half> &costs, int x, int y, int i) {
	return __half2float(costs(x>>2, y>>2 + (costs.height()/16)*i));
}

template <int SIZE>
__device__ inline void writeCost(float (&costs)[SIZE][WARP_SIZE*2], int i, float cost) {
	costs[i][threadIdx.x + threadIdx.y*WARP_SIZE] = cost;
}

template <int SIZE>
__device__ inline float readCost(const float (&costs)[SIZE][WARP_SIZE*2], int i) {
	return costs[i][threadIdx.x + threadIdx.y*WARP_SIZE];
}

__device__ inline void writeCost(float (&costs)[WARP_SIZE*2], float cost) {
	costs[threadIdx.x + threadIdx.y*WARP_SIZE] = cost;
}

__device__ inline float readCost(const float (&costs)[WARP_SIZE*2]) {
	return costs[threadIdx.x + threadIdx.y*WARP_SIZE];
}

template<int COR_STEPS> 
__global__ void corresponding_colour_epi_kernel(
		TextureObject<uchar4> colour1,
		TextureObject<uchar4> colour2,
        TextureObject<float> d1,
        TextureObject<float> d2,
        TextureObject<short2> screenOut,
		TextureObject<float> conf,
		//TextureObject<uint8_t> mask,
		//TextureObject<half> costs,
        float4x4 pose,
        Camera cam1,
        Camera cam2, ftl::cuda::MvMLSParams params) {

	__shared__ float match_costs[COR_STEPS][WARP_SIZE*2];
	__shared__ float aggr_costs_f[COR_STEPS][WARP_SIZE*2];
	__shared__ float aggr_costs_b[COR_STEPS][WARP_SIZE*2];
	__shared__ float min_costs[WARP_SIZE*2];
	
	const int2 pt = block4x4<2>();

	// FIXME: Don't return like this due to warp operations
    if (pt.x >= 0 && pt.y >=0 && pt.x < colour1.width() && pt.y < colour1.height()) {
		screenOut(pt) = make_short2(-1,-1);
		conf(pt) = PINF;
    
		const float depth1 = d1.tex2D(pt);
		// TODO: If all depths within a half warp here are bad then can return
		// early.
		//if (__ballot_sync(FULL_MASK, depth1 > cam1.minDepth && depth1 < cam1.maxDepth) & ((threadIdx.x/16 == 1) ? 0xFFFF0000 : 0xFFFF) == 0) return;

        short2 bestScreen = make_short2(-1,-1);
		float bestcost = PINF;
		float bestcost2 = PINF;
		
		const float3 camPosOrigin = pose * cam1.screenToCam(pt.x,pt.y,depth1);
        const float2 lineOrigin = cam2.camToScreen<float2>(camPosOrigin);
        const float3 camPosDistant = pose * cam1.screenToCam(pt.x,pt.y,depth1 + 0.4f);
        const float2 lineDistant = cam2.camToScreen<float2>(camPosDistant);
		const float lineM = (lineDistant.y - lineOrigin.y) / (lineDistant.x - lineOrigin.x);
		const float depthM = 0.4f / (lineDistant.x - lineOrigin.x);
		//const float sub_pixel = depthM / (params.sub_pixel * depth1);
		const float sub_pixel = params.sub_pixel; //depthM / (((depth1*depth1) / (cam1.fx * cam1.baseline)) / float(COR_STEPS/2));
        float2 linePos;
        linePos.x = lineOrigin.x - float((COR_STEPS/2)) * sub_pixel;
        linePos.y = lineOrigin.y - (float((COR_STEPS/2)) * lineM * sub_pixel);

		// generate census for camera1
		/*float4 c1_px_cy = colour1.tex2D(float(x)-0.5f,float(y));
		float4 c1_nx_cy = colour1.tex2D(float(x)+0.5f,float(y));
		float4 c1_cx_py = colour1.tex2D(float(x),float(y)-0.5f);
		float4 c1_cx_ny = colour1.tex2D(float(x),float(y)+0.5f);*/
		float4 c1 = colour1.tex2D(float(pt.x)+0.5f,float(pt.y)+0.5f);

		int bestStep = COR_STEPS/2;

		float avgcost = 0.0f;
		
		//writeCost(min_costs, PINF);

		// Generate matching costs
		#pragma unroll
        for (int i=0; i<COR_STEPS; ++i) {	
			//writeCost(aggr_costs_f, i, 0.0f);
			//writeCost(aggr_costs_b, i, 0.0f);

			//    generate census at that location+0.5f
			float4 c2 = colour2.tex2D(linePos);
			/*float d = min(
				distance(c1_px_cy-c1_nx_cy, c2-c1_nx_cy),
				distance(c1_cx_py-c1_cx_ny, c2-c1_cx_ny)
			);*/
			float d = distance(c1,c2);
			//d *= d;
			//d = (ftl::cuda::halfWarpSum(d*d) / 16.0f);

			if (linePos.x < 0 || linePos.x >= cam2.width || linePos.y < 0 || linePos.y >= cam2.height) d = PINF;
			//if (ftl::cuda::halfWarpMax(d) == PINF) d = PINF;

			writeCost(match_costs, i, d);
			bestcost = min(bestcost, d);
			avgcost += d;

			// Move to next correspondence check point
            linePos.x += sub_pixel*1.0f;
            linePos.y += sub_pixel*lineM;
		}

		writeCost(min_costs, bestcost);
		bestcost = PINF;
		//avgcost = avgcost / float(COR_STEPS);

		__syncwarp();

		float confidence = 0.0f;

		// Forward aggregate matching costs
        for (int i=0; i<COR_STEPS; ++i) {
			float cost = readCost(match_costs,i);
			float dcost = cost - ((avgcost-cost)/float(COR_STEPS-1));
			float variance = (dcost < 0.0f) ? square(dcost) : 1.0f;
			cost /= sqrt(variance);
			cost = (ftl::cuda::halfWarpSum(cost*cost) / 16.0f);

			cost = ftl::cuda::halfWarpSum(variance) / 16.0f;

			//float cost2 = (i>0) ? readCost(match_costs,i-1) + params.P1 : PINF;
			//float cost3 = (i<COR_STEPS-1) ? readCost(match_costs,i+1) + params.P1 : PINF;
			//float cost4 = readCost(min_costs) + params.P2;

			// Instead of the above...
			// Add the min costs of neighbours +1, 0 and -1 steps

			//float mincost = min(cost,min(cost2,cost3));

			//cost += (ftl::cuda::halfWarpSum(mincost) - mincost) / 15.0f; // - readCost(min_costs); // / 15.0f;
			//cost /= 2.0f;
			//writeCost(aggr_costs_f, i, cost);
			//writeCost(min_costs, min(readCost(min_costs),cost));

			bestStep = (cost < bestcost) ? i : bestStep;
			bestcost = min(cost, bestcost);
			//avgcost += cost;
		}

		confidence = sqrt(confidence / float(COR_STEPS));
		//confidence = max(0.0f, (fabsf(avgcost-bestcost) / confidence) - 1.0f); // / (3.0f * confidence);
		bestcost = sqrt(bestcost);

		// Backward aggregate costs
		//for (int i=COR_STEPS-1; i>=0; --i) {
			/*float cost = ftl::cuda::halfWarpSum(readCost(match_costs,i));
			if (i<COR_STEPS-1) cost += ftl::cuda::halfWarpSum(readCost(aggr_costs_b,i+1)+15.0f*10.0f) - readCost(aggr_costs_b,i+1);
			if (i<COR_STEPS-2) cost += ftl::cuda::halfWarpSum(readCost(aggr_costs_b,i+2)+15.0f*30.0f) - readCost(aggr_costs_b,i+2);
			float N = (i<COR_STEPS-2) ? 46.0f : (i<COR_STEPS-1) ? 31.0f : 16.0f;
			writeCost(aggr_costs_b, i, cost / N);*/
		//}

		//__syncwarp();

		// Find best cost and use that
		/*for (int i=0; i<COR_STEPS; ++i) {
			float cost = readCost(aggr_costs_f,i) + readCost(aggr_costs_b,i);
			bestStep = (cost < bestcost) ? i : bestStep;
			bestcost = min(cost, bestcost);
		}*/

		float bestadjust = float(bestStep-(COR_STEPS/2))*depthM*sub_pixel;
		bestScreen = make_short2(lineOrigin.x + float(bestStep-(COR_STEPS/2))*sub_pixel + 0.5f,
			lineOrigin.y + float(bestStep-(COR_STEPS/2))*lineM*sub_pixel + 0.5f);

        // Detect matches to boundaries, and discard those
        //uint stepMask = 1 << bestStep;
		//if ((stepMask & badMask) || (stepMask & (badMask << 1)) || (stepMask & (badMask >> 1))) bestcost = PINF;

		// TODO: Validate depth?

		// Check that no boundaries were matched in 4x4 block
		//bestcost = ftl::cuda::halfWarpMax(bestcost);

		//float confidence = (avgcost == PINF) ? 1.0f : (bestcost / (avgcost/float(COR_STEPS)));
		
		// If a match was found then record it
		if (depth1 > cam1.minDepth && depth1 < cam1.maxDepth) {
			// Delay making the depth change until later.

			//if (y == 200) printf("Cost: %d\n", bestScreen.x);

			/*if (x % 2 == 0 && y % 2 == 0) {
				const float depthM2 = (camPosDistant.z - camPosOrigin.z) / (lineDistant.x - lineOrigin.x);
				if (fabs(d2.tex2D(bestScreen.x, bestScreen.y) - (camPosOrigin.z + float(bestStep-(COR_STEPS/2))*depthM2*sub_pixel)) < 0.1f*camPosOrigin.z) {
					float4 c2 = colour2.tex2D(float(bestScreen.x)+0.5f, float(bestScreen.y)+0.5f);
					colour1(x,y) = make_uchar4(c2.x, c2.y, c2.z, 0.0f);
				}
			}*/
			
			const float depthM2 = (camPosDistant.z - camPosOrigin.z) / (lineDistant.x - lineOrigin.x);
			if (fabsf(d2.tex2D(bestScreen) - (camPosOrigin.z + float(bestStep-(COR_STEPS/2))*depthM2*sub_pixel)) < 0.1f*camPosOrigin.z) {
				if (bestcost < PINF && bestStep != 0 && bestStep != COR_STEPS-1) {
					conf(pt) = bestcost;
					//colour1(x,y) = make_uchar4(bestc2.x, bestc2.y, bestc2.z, 0.0f);
					//mask(x,y) = mask(x,y) | Mask::kMask_Correspondence;
					screenOut(pt) = bestScreen;
				}
			}
		}

		// Doesn't work
		//if (depth1 > cam1.minDepth && depth1 < cam1.maxDepth && bestcost > 0.75f) {
		//	mask(x,y) = mask(x,y) | Mask::kMask_Bad;
		//}
    }
}

template <int COR_STEPS>
__global__ void aggregate_colour_costs_kernel(
		TextureObject<float> d1,
		TextureObject<float> d2,
		TextureObject<half> costs1,
		TextureObject<half> costs2,
		TextureObject<short2> screenOut,
		TextureObject<float> conf,
		float4x4 pose,
		Camera cam1,
		Camera cam2,
		ftl::cuda::MvMLSParams params
	) {
	
	//const int tid = (threadIdx.x + threadIdx.y * blockDim.x);
	const int x = (blockIdx.x*8 + (threadIdx.x%4) + 4*(threadIdx.x / 16)); // / WARP_SIZE;
	const int y = blockIdx.y*8 + threadIdx.x/4 + 4*threadIdx.y;

	// FIXME: Don't return like this due to warp operations
    if (x >= 0 && y >=0 && x < d1.width() && y < d1.height()) {
		screenOut(x,y) = make_short2(-1,-1);
		conf(x,y) = 0.0f;
    
		const float depth1 = d1.tex2D(x,y);
		// TODO: If all depths within a half warp here are bad then can return
		// early.
		if (__ballot_sync(FULL_MASK, depth1 > cam1.minDepth && depth1 < cam1.maxDepth) & ((threadIdx.x/16 == 1) ? 0xFFFF0000 : 0xFFFF) == 0) return;

        short2 bestScreen = make_short2(-1,-1);
		float bestcost = PINF;
		float bestcost2 = PINF;
		
		const float3 camPosOrigin = pose * cam1.screenToCam(x,y,depth1);
        const float2 lineOrigin = cam2.camToScreen<float2>(camPosOrigin);
        const float3 camPosDistant = pose * cam1.screenToCam(x,y,depth1 + 0.4f);
        const float2 lineDistant = cam2.camToScreen<float2>(camPosDistant);
		const float lineM = (lineDistant.y - lineOrigin.y) / (lineDistant.x - lineOrigin.x);
		const float depthM = 0.4f / (lineDistant.x - lineOrigin.x);
		const float depthM2 = (camPosDistant.z - camPosOrigin.z) / (lineDistant.x - lineOrigin.x);
		const float sub_pixel = depthM / (((depth1*depth1) / (cam1.fx * cam1.baseline)) / float(COR_STEPS/2));
		float depthPos2 = camPosOrigin.z - (float((COR_STEPS/2)) * depthM2 * sub_pixel);
        float2 linePos;
        linePos.x = lineOrigin.x - float((COR_STEPS/2)) * sub_pixel;
        linePos.y = lineOrigin.y - (float((COR_STEPS/2)) * lineM * sub_pixel);

        uint badMask = 0;
        int bestStep = COR_STEPS/2;
		//float bestSource = 0.0f;

		// Scan from -COR_STEPS/2 to +COR_STEPS/2, where COR_STEPS is the number
		// of pixels to check in the second camera for the current first camera
		// pixel.

		#pragma unroll
        for (int i=0; i<COR_STEPS; ++i) {			
			//    generate census at that location+0.5f
			float depth2 = d2.tex2D(int(linePos.x+0.5f), int(linePos.y+0.5f));
			const float sub_pixel2 = (((depth2*depth2) / (cam2.fx * cam2.baseline)) / float(COR_STEPS/2));
			int index = int((depthPos2-depth2) / sub_pixel2) + COR_STEPS/2;

			float c1 = readCost(costs1, x, y, i);
			float c2 = (depthPos2 > cam2.minDepth && depthPos2 < cam2.maxDepth && index >= 0 && index < COR_STEPS && linePos.x >= 0.0f && linePos.y >= 0.0f && linePos.x < d2.width()-1 && linePos.y < d2.height()-1) ? readCost(costs2, int(linePos.x+0.5f), int(linePos.y+0.5f), index) : PINF;
			//writeCost(costs1, x, y, i, c1+c2);

			//if (index >= 0 && index < COR_STEPS && y == 200) printf("Index = %f\n", sub_pixel);
			float cost = c1+c2;

			// Record the best result
            bestStep = (cost < bestcost) ? i : bestStep;
			bestScreen = (cost < bestcost) ? make_short2(linePos.x+0.5f,linePos.y+0.5f) : bestScreen;
			//bestc2 = (cost < bestcost) ? c2 : bestc2;
			bestcost = min(bestcost, cost);
                
			
			// Move to next correspondence check point
			depthPos2 += sub_pixel*depthM2;
            linePos.x += sub_pixel*1.0f;
            linePos.y += sub_pixel*lineM;
        }

        float bestadjust = float(bestStep-(COR_STEPS/2))*depthM*sub_pixel;

        // Detect matches to boundaries, and discard those
        //uint stepMask = 1 << bestStep;
		//if ((stepMask & badMask) || (stepMask & (badMask << 1)) || (stepMask & (badMask >> 1))) bestcost = PINF;

		// TODO: Validate depth?

		// Check that no boundaries were matched in 4x4 block
		//bestcost = ftl::cuda::halfWarpMax(bestcost);

		//float confidence = 1.0f - (bestcost2 / bestcost);
		
		// If a match was found then record it
		if (depth1 > cam1.minDepth && depth1 < cam1.maxDepth && bestcost < PINF) {
			// Delay making the depth change until later.
			
			conf(x,y) = bestadjust;
			//colour1(x,y) = make_uchar4(bestc2.x, bestc2.y, bestc2.z, 0.0f);
			//mask(x,y) = mask(x,y) | Mask::kMask_Correspondence;
			screenOut(x,y) = bestScreen;
		}

		// Doesn't work
		//if (depth1 > cam1.minDepth && depth1 < cam1.maxDepth && bestcost > 0.75f) {
		//	mask(x,y) = mask(x,y) | Mask::kMask_Bad;
		//}
    }
}

/**
 * Non-epipolar census search.
 */
template <int RADIUS>
__global__ void corresponding_colour_kernel(
		TextureObject<uchar4> colour1,
		TextureObject<uchar4> colour2,
		TextureObject<short2> screen,
		float4x4 pose,
		Camera cam1,
		Camera cam2, ftl::cuda::MvMLSParams params) {

	const int x = (blockIdx.x*8 + (threadIdx.x%4) + 4*(threadIdx.x / 16));
	const int y = blockIdx.y*8 + threadIdx.x/4 + 4*threadIdx.y;

	if (x >= 0 && y >=0 && x < screen.width() && y < screen.height()) {
		// generate census for camera1
		float4 c = colour1.tex2D(float(x+0.5f),float(y+0.5f));
		int3 cen1;
		cen1.x = halfWarpCensus(c.x);
		cen1.y = halfWarpCensus(c.y);
		cen1.z = halfWarpCensus(c.z);

		short2 corPos = screen.tex2D(x,y);
		if (ftl::cuda::halfWarpMin(int(corPos.x)) < 0) return;

		short2 bestScreen = corPos;
		float bestcost = PINF;

		// For a perturbation around current correspondence..
		for (int v=-RADIUS; v<=RADIUS; ++v) {
		for (int u=-RADIUS; u<=RADIUS; ++u) {
			//    generate census at that location
			float4 c = colour2.tex2D(float(corPos.x+u), float(corPos.y+v));
			int3 cen2;
			cen2.x = halfWarpCensus(c.x);
			cen2.y = halfWarpCensus(c.y);
			cen2.z = halfWarpCensus(c.z);

			//    count bit differences
			int count = __popc(cen1.x ^ cen2.x) + __popc(cen1.y ^ cen2.y) + __popc(cen1.z ^ cen2.z);

			//    generate cost from bit diffs and amount of movement
			float cost = float(count) / 48.0f;
			cost += 1.0f - min(1.0f, sqrt(float(v*v) + float(u*u)) / float(3*RADIUS));
			bestScreen = (cost < bestcost) ? make_short2(corPos.x+u, corPos.y+v) : bestScreen;
			bestcost = min(cost, bestcost);
		}
		}

		// Adjust correspondence to min cost location
		screen(x,y) = bestScreen;
	}
}


void ftl::cuda::correspondence(
		TextureObject<uchar4> &c1,
		TextureObject<uchar4> &c2,
		TextureObject<float> &d1,
        TextureObject<float> &d2,
		TextureObject<short2> &screen,
		TextureObject<float> &conf,
		//TextureObject<uint8_t> &mask,
		//TextureObject<half> &costs,
		float4x4 &pose2,
		const Camera &cam1,
		const Camera &cam2, const MvMLSParams &params,
		int radius,
		cudaStream_t stream) {

	//const dim3 gridSize((d1.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (d1.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	//const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	const dim3 gridSize((c1.width() + 8 - 1)/8, (c1.height() + 8 - 1)/8);
	const dim3 blockSize(WARP_SIZE, 2);


	switch (radius) {
	case 32: corresponding_colour_epi_kernel<32><<<gridSize, blockSize, 0, stream>>>(c1, c2, d1, d2, screen, conf, pose2, cam1, cam2, params); break;
	case 16: corresponding_colour_epi_kernel<16><<<gridSize, blockSize, 0, stream>>>(c1, c2, d1, d2, screen, conf, pose2, cam1, cam2, params); break;
	case 8: corresponding_colour_epi_kernel<8><<<gridSize, blockSize, 0, stream>>>(c1, c2, d1, d2, screen, conf, pose2, cam1, cam2, params); break;
	case 4: corresponding_colour_epi_kernel<4><<<gridSize, blockSize, 0, stream>>>(c1, c2, d1, d2, screen, conf, pose2, cam1, cam2, params); break;
	case 3: corresponding_colour_epi_kernel<3><<<gridSize, blockSize, 0, stream>>>(c1, c2, d1, d2, screen, conf, pose2, cam1, cam2, params); break;
	case 2: corresponding_colour_epi_kernel<2><<<gridSize, blockSize, 0, stream>>>(c1, c2, d1, d2, screen, conf, pose2, cam1, cam2, params); break;
	//case 1: corresponding_colour_epi_kernel<1><<<gridSize, blockSize, 0, stream>>>(c1, c2, d1, d2, costs, pose2, cam1, cam2, params); break;
	}

	cudaSafeCall( cudaGetLastError() );
}

void ftl::cuda::aggregate_colour_costs(
		TextureObject<float> &d1,
		TextureObject<float> &d2,
		TextureObject<half> &costs1,
		TextureObject<half> &costs2,
		TextureObject<short2> &screen,
		TextureObject<float> &conf,
		float4x4 &pose2,
		const Camera &cam1,
		const Camera &cam2, const MvMLSParams &params,
		int radius,
		cudaStream_t stream) {

	//const dim3 gridSize((d1.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (d1.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	//const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	const dim3 gridSize((d1.width() + 1), (d1.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(WARP_SIZE, 2);


	switch (radius) {
	case 32: aggregate_colour_costs_kernel<32><<<gridSize, blockSize, 0, stream>>>(d1, d2, costs1, costs2, screen, conf, pose2, cam1, cam2, params); break;
	case 16: aggregate_colour_costs_kernel<16><<<gridSize, blockSize, 0, stream>>>(d1, d2, costs1, costs2, screen, conf, pose2, cam1, cam2, params); break;
	case 8: aggregate_colour_costs_kernel<8><<<gridSize, blockSize, 0, stream>>>(d1, d2, costs1, costs2, screen, conf, pose2, cam1, cam2, params); break;
	case 4: aggregate_colour_costs_kernel<4><<<gridSize, blockSize, 0, stream>>>(d1, d2, costs1, costs2, screen, conf, pose2, cam1, cam2, params); break;
	case 3: aggregate_colour_costs_kernel<3><<<gridSize, blockSize, 0, stream>>>(d1, d2, costs1, costs2, screen, conf, pose2, cam1, cam2, params); break;
	case 2: aggregate_colour_costs_kernel<2><<<gridSize, blockSize, 0, stream>>>(d1, d2, costs1, costs2, screen, conf, pose2, cam1, cam2, params); break;
	//case 1: aggregate_colour_costs_kernel<1><<<gridSize, blockSize, 0, stream>>>(d1, d2, costs1, costs2, screen, conf, pose2, cam1, cam2, params); break;
	}

	cudaSafeCall( cudaGetLastError() );
}
