#include "integrators.hpp"
//#include <ftl/ray_cast_params.hpp>
#include <vector_types.h>
#include <cuda_runtime.h>
#include <ftl/cuda_matrix_util.hpp>
#include <ftl/cuda_util.hpp>
#include <ftl/cuda_common.hpp>

#define T_PER_BLOCK 8
#define NUM_CUDA_BLOCKS		10000
#define WARP_SIZE 32

using ftl::voxhash::HashData;
using ftl::voxhash::HashParams;
using ftl::voxhash::Voxel;
using ftl::voxhash::HashEntry;
using ftl::voxhash::HashEntryHead;
using ftl::voxhash::FREE_ENTRY;

extern __constant__ ftl::voxhash::DepthCameraCUDA c_cameras[MAX_CAMERAS];
extern __constant__ HashParams c_hashParams;

__device__ float4 make_float4(uchar4 c) {
	return make_float4(static_cast<float>(c.x), static_cast<float>(c.y), static_cast<float>(c.z), static_cast<float>(c.w));
}

__device__ float colourDistance(const uchar4 &c1, const uchar3 &c2) {
	float x = c1.x-c2.x;
	float y = c1.y-c2.y;
	float z = c1.z-c2.z;
	return x*x + y*y + z*z;
}

/*
 * Kim, K., Chalidabhongse, T. H., Harwood, D., & Davis, L. (2005).
 * Real-time foreground-background segmentation using codebook model.
 * Real-Time Imaging. https://doi.org/10.1016/j.rti.2004.12.004
 */
__device__ bool colordiff(const uchar4 &pa, const uchar3 &pb, float epsilon) {
	float x_2 = pb.x * pb.x + pb.y * pb.y + pb.z * pb.z;
	float v_2 = pa.x * pa.x + pa.y * pa.y + pa.z * pa.z;
	float xv_2 = pow(pb.x * pa.x + pb.y * pa.y + pb.z * pa.z, 2);
	float p_2 = xv_2 / v_2;
	return sqrt(x_2 - p_2) < epsilon;
}

/*
 * Guennebaud, G.; Gross, M. Algebraic point set surfaces. ACMTransactions on Graphics Vol. 26, No. 3, Article No. 23, 2007.
 * Used in: FusionMLS: Highly dynamic 3D reconstruction with consumer-grade RGB-D cameras
 *     r = distance between points
 *     h = smoothing parameter in meters (default 4cm)
 */
__device__ float spatialWeighting(float r) {
	const float h = c_hashParams.m_spatialSmoothing;
	if (r >= h) return 0.0f;
	float rh = r / h;
	rh = 1.0f - rh*rh;
	return rh*rh*rh*rh;
}


__global__ void integrateDepthMapsKernel(HashData hashData, HashParams hashParams, int numcams) {
	__shared__ uint all_warp_ballot;
	__shared__ uint voxels[16];

	const uint i = threadIdx.x;	//inside of an SDF block
	const int3 po = make_int3(hashData.delinearizeVoxelIndex(i));

	// Stride over all allocated blocks
	for (int bi=blockIdx.x; bi<*hashData.d_hashCompactifiedCounter; bi+=NUM_CUDA_BLOCKS) {

	//TODO check if we should load this in shared memory
	//HashEntryHead entry = hashData.d_hashCompactified[bi]->head;

	int3 pi_base = hashData.SDFBlockToVirtualVoxelPos(make_int3(hashData.d_hashCompactified[bi]->head.posXYZ));

	//uint idx = entry.offset + i;
	int3 pi = pi_base + po;
	float3 pfb = hashData.virtualVoxelPosToWorld(pi);
	int count = 0;
	//float camdepths[MAX_CAMERAS];

	Voxel oldVoxel; // = hashData.d_SDFBlocks[idx];
	hashData.deleteVoxel(oldVoxel);

	for (uint cam=0; cam<numcams; ++cam) {
		const ftl::voxhash::DepthCameraCUDA &camera = c_cameras[cam];
	
		float3 pf = camera.poseInverse * pfb;
		uint2 screenPos = make_uint2(camera.params.cameraToKinectScreenInt(pf));

		// For this voxel in hash, get its screen position and check it is on screen
		if (screenPos.x < camera.params.m_imageWidth && screenPos.y < camera.params.m_imageHeight) {	//on screen

			//float depth = g_InputDepth[screenPos];
			float depth = tex2D<float>(camera.depth, screenPos.x, screenPos.y);
			//if (depth > 20.0f) return;

			//uchar4 color  = make_uchar4(0, 0, 0, 0);
			//if (cameraData.d_colorData) {
				//color = (cam == 0) ? make_uchar4(255,0,0,255) : make_uchar4(0,0,255,255);
				//color = tex2D<uchar4>(camera.colour, screenPos.x, screenPos.y);
				//color = bilinearFilterColor(cameraData.cameraToKinectScreenFloat(pf));
			//}

			//printf("screen pos %d\n", color.x);
			//return;

			// TODO:(Nick) Accumulate weighted positions
			// TODO:(Nick) Accumulate weighted normals
			// TODO:(Nick) Accumulate weights

			// Depth is within accepted max distance from camera
			if (depth > 0.01f && depth < hashParams.m_maxIntegrationDistance) { // valid depth and color (Nick: removed colour check)
				//camdepths[count] = depth;
				++count;

				// Calculate SDF of this voxel wrt the depth map value
				float sdf = depth - pf.z;
				float truncation = hashData.getTruncation(depth);
				float depthZeroOne = camera.params.cameraToKinectProjZ(depth);

				// Is this voxel close enough to cam for depth map value
				// CHECK Nick: If is too close then free space violation so remove?
				if (sdf > -truncation) // && depthZeroOne >= 0.0f && depthZeroOne <= 1.0f) //check if in truncation range should already be made in depth map computation
				{
					float weightUpdate = max(hashParams.m_integrationWeightSample * 1.5f * (1.0f-depthZeroOne), 1.0f);

					Voxel curr;	//construct current voxel
					curr.sdf = sdf;
					curr.weight = weightUpdate;
					//curr.color = make_uchar3(color.x, color.y, color.z);


					//if (entry.flags != cameraParams.flags & 0xFF) {
					//	entry.flags = cameraParams.flags & 0xFF;
						//hashData.d_SDFBlocks[idx].color = make_uchar3(0,0,0);
					//}
					
					Voxel newVoxel;
					//if (color.x == MINF) hashData.combineVoxelDepthOnly(hashData.d_SDFBlocks[idx], curr, newVoxel);
					//else hashData.combineVoxel(hashData.d_SDFBlocks[idx], curr, newVoxel);
					hashData.combineVoxel(oldVoxel, curr, newVoxel);

					oldVoxel = newVoxel;

					//Voxel prev = getVoxel(g_SDFBlocksSDFUAV, g_SDFBlocksRGBWUAV, idx);
					//Voxel newVoxel = combineVoxel(curr, prev);
					//setVoxel(g_SDFBlocksSDFUAV, g_SDFBlocksRGBWUAV, idx, newVoxel);
				}
			} else {
				// Depth is invalid so what to do here?
				// TODO(Nick) Use past voxel if available (set weight from 0 to 1)

				// Naive: need to know if this is a foreground voxel
				//bool coldist = colordiff(color, hashData.d_SDFBlocks[idx].color, 5.0f);
				//if (!coldist) ++count;

			}
		}
	}

	// Calculate voxel sign values across a warp
	int warpNum = i / WARP_SIZE;
	//uint ballot_result = __ballot_sync(0xFFFFFFFF, (oldVoxel.sdf >= 0.0f) ? 0 : 1);
	uint ballot_result = __ballot_sync(0xFFFFFFFF, (fabs(oldVoxel.sdf) <= hashParams.m_virtualVoxelSize && oldVoxel.weight > 0) ? 1 : 0);

	// Aggregate each warp result into voxel mask
	if (i % WARP_SIZE == 0) {
		voxels[warpNum] = ballot_result;
	}

	__syncthreads();

	// Work out if block is occupied or not and save voxel masks
	// TODO:(Nick) Is it faster to do this in a separate garbage kernel?
	if (i < 16) {
		const uint v = voxels[i];
		hashData.d_hashCompactified[bi]->voxels[i] = v;
		const uint mask = 0x0000FFFF;
		uint b1 = __ballot_sync(mask, v == 0xFFFFFFFF);
		uint b2 = __ballot_sync(mask, v == 0);
		if (i == 0) {
			if (b1 != mask && b2 != mask) hashData.d_hashCompactified[bi]->head.flags |= ftl::voxhash::kFlagSurface;
			else hashData.d_hashCompactified[bi]->head.flags &= ~ftl::voxhash::kFlagSurface;
		}
	}

	}
}

#define WINDOW_RADIUS 1
#define PATCH_SIZE 32

__global__ void integrateMLSKernel(HashData hashData, HashParams hashParams, int numcams) {
	__shared__ uint voxels[16];

	const uint i = threadIdx.x;	//inside of an SDF block
	const int3 po = make_int3(hashData.delinearizeVoxelIndex(i));
	const int warpNum = i / WARP_SIZE;
	const int lane = i % WARP_SIZE;

	// Stride over all allocated blocks
	for (int bi=blockIdx.x; bi<*hashData.d_hashCompactifiedCounter; bi+=NUM_CUDA_BLOCKS) {

	//TODO check if we should load this in shared memory
	//HashEntryHead entry = hashData.d_hashCompactified[bi]->head;

	const int3 pi_base = hashData.SDFBlockToVirtualVoxelPos(make_int3(hashData.d_hashCompactified[bi]->head.posXYZ));

	//uint idx = entry.offset + i;
	const int3 pi = pi_base + po;
	const float3 pfb = hashData.virtualVoxelPosToWorld(pi);
	//int count = 0;
	//float camdepths[MAX_CAMERAS];

	//Voxel oldVoxel; // = hashData.d_SDFBlocks[idx];
	//hashData.deleteVoxel(oldVoxel);

	//float3 awpos = make_float3(0.0f);
	//float3 awnorm = make_float3(0.0f);
	//float aweights = 0.0f;
	float sdf = 0.0f;
	float weights = 0.0f;

	// Preload depth values
	// 1. Find min and max screen positions
	// 2. Subtract/Add WINDOW_RADIUS to min/max
	// ... check that the buffer is not too small to cover this
	// ... if buffer not big enough then don't buffer at all.
	// 3. Populate shared mem depth map buffer using all threads
	// 4. Adjust window lookups to use shared mem buffer

	//uint cam=0;
	for (uint cam=0; cam<numcams; ++cam) {
		const ftl::voxhash::DepthCameraCUDA &camera = c_cameras[cam];
		const uint height = camera.params.m_imageHeight;
		const uint width = camera.params.m_imageWidth;
	
		const float3 pf = camera.poseInverse * pfb;
		const uint2 screenPos = make_uint2(camera.params.cameraToKinectScreenInt(pf));

		//float3 wpos = make_float3(0.0f);
		float3 wnorm = make_float3(0.0f);
		

		#pragma unroll
		for (int v=-WINDOW_RADIUS; v<=WINDOW_RADIUS; ++v) {
			for (int u=-WINDOW_RADIUS; u<=WINDOW_RADIUS; ++u) {
				if (screenPos.x+u < width && screenPos.y+v < height) {	//on screen
					float4 depth = tex2D<float4>(camera.points, screenPos.x+u, screenPos.y+v);
					if (depth.z == MINF) continue;

					//float4 normal = tex2D<float4>(camera.normal, screenPos.x+u, screenPos.y+v);
					const float3 camPos = camera.poseInverse * make_float3(depth); //camera.pose * camera.params.kinectDepthToSkeleton(screenPos.x+u, screenPos.y+v, depth);
					const float weight = spatialWeighting(length(pf - camPos));

					//wpos += weight*worldPos;
					sdf += weight*(camPos.z - pf.z);
					//sdf += camPos.z - pf.z;
					//wnorm += weight*make_float3(normal);
					//weights += 1.0f;	
					weights += weight;			
				}
			}
		}

		//awpos += wpos;
		//aweights += weights;
	}

	//awpos /= aweights;
	//wnorm /= weights;

	sdf /= weights;

	//float sdf = (aweights == 0.0f) ? MINF : length(pfb - awpos);
	//float sdf = wnorm.x * (pfb.x - wpos.x) + wnorm.y * (pfb.y - wpos.y) + wnorm.z * (pfb.z - wpos.z);

	//printf("WEIGHTS: %f\n", weights);

	//if (weights < 0.00001f) sdf = 0.0f;

	// Calculate voxel sign values across a warp
	int warpNum = i / WARP_SIZE;

	//uint solid_ballot = __ballot_sync(0xFFFFFFFF, (fabs(sdf) < hashParams.m_virtualVoxelSize && aweights >= 0.5f) ? 1 : 0);
	//uint solid_ballot = __ballot_sync(0xFFFFFFFF, (fabs(sdf) <= hashParams.m_virtualVoxelSize) ? 1 : 0);
	//uint solid_ballot = __ballot_sync(0xFFFFFFFF, (aweights >= 0.0f) ? 1 : 0);
	uint solid_ballot = __ballot_sync(0xFFFFFFFF, (sdf < 0.0f ) ? 1 : 0);

	// Aggregate each warp result into voxel mask
	if (i % WARP_SIZE == 0) {
		voxels[warpNum] = solid_ballot;
		//valid[warpNum] = valid_ballot;
	}

	__syncthreads();

	// Work out if block is occupied or not and save voxel masks
	// TODO:(Nick) Is it faster to do this in a separate garbage kernel?
	if (i < 16) {
		const uint v = voxels[i];
		hashData.d_hashCompactified[bi]->voxels[i] = v;
		//hashData.d_hashCompactified[bi]->validity[i] = valid[i];
		const uint mask = 0x0000FFFF;
		uint b1 = __ballot_sync(mask, v == 0xFFFFFFFF);
		uint b2 = __ballot_sync(mask, v == 0);
		if (i == 0) {
			if (b1 != mask && b2 != mask) hashData.d_hashCompactified[bi]->head.flags |= ftl::voxhash::kFlagSurface;
			else hashData.d_hashCompactified[bi]->head.flags &= ~ftl::voxhash::kFlagSurface;
		}
	}

	}
}



void ftl::cuda::integrateDepthMaps(HashData& hashData, const HashParams& hashParams, int numcams, cudaStream_t stream) {
const unsigned int threadsPerBlock = SDF_BLOCK_SIZE*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE;
const dim3 gridSize(NUM_CUDA_BLOCKS, 1);
const dim3 blockSize(threadsPerBlock, 1);

//if (hashParams.m_numOccupiedBlocks > 0) {	//this guard is important if there is no depth in the current frame (i.e., no blocks were allocated)
	integrateMLSKernel << <gridSize, blockSize, 0, stream >> >(hashData, hashParams, numcams);
//}

//cudaSafeCall( cudaGetLastError() );
#ifdef _DEBUG
cudaSafeCall(cudaDeviceSynchronize());
//cutilCheckMsg(__FUNCTION__);
#endif
}
