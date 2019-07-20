#include "integrators.hpp"
//#include <ftl/ray_cast_params.hpp>
#include <vector_types.h>
#include <cuda_runtime.h>
#include <ftl/cuda_matrix_util.hpp>
#include <ftl/cuda_util.hpp>
#include <ftl/cuda_common.hpp>

#define T_PER_BLOCK 8

using ftl::voxhash::HashData;
using ftl::voxhash::HashParams;
using ftl::voxhash::Voxel;
using ftl::voxhash::HashEntry;
using ftl::voxhash::HashEntryHead;
using ftl::voxhash::FREE_ENTRY;

__device__ float4 make_float4(uchar4 c) {
	return make_float4(static_cast<float>(c.x), static_cast<float>(c.y), static_cast<float>(c.z), static_cast<float>(c.w));
}

inline __device__ uchar4 bilinearFilterColor(const DepthCameraParams& cameraParams, const float2& screenPos, cudaTextureObject_t colorTextureRef) {
	//const DepthCameraParams& cameraParams = c_depthCameraParams;
	const int imageWidth = cameraParams.m_imageWidth;
	const int imageHeight = cameraParams.m_imageHeight;
	const int2 p00 = make_int2(screenPos.x+0.5f, screenPos.y+0.5f);
	const int2 dir = sign(make_float2(screenPos.x - p00.x, screenPos.y - p00.y));

	const int2 p01 = p00 + make_int2(0.0f, dir.y);
	const int2 p10 = p00 + make_int2(dir.x, 0.0f);
	const int2 p11 = p00 + make_int2(dir.x, dir.y);

	const float alpha = (screenPos.x - p00.x)*dir.x;
	const float beta  = (screenPos.y - p00.y)*dir.y;

	float4 s0 = make_float4(0.0f, 0.0f, 0.0f, 0.0f); float w0 = 0.0f;
	if(p00.x >= 0 && p00.x < imageWidth && p00.y >= 0 && p00.y < imageHeight) { uchar4 v00 = tex2D<uchar4>(colorTextureRef, p00.x, p00.y); if(v00.x != 0) { s0 += (1.0f-alpha)*make_float4(v00); w0 += (1.0f-alpha); } }
	if(p10.x >= 0 && p10.x < imageWidth && p10.y >= 0 && p10.y < imageHeight) { uchar4 v10 = tex2D<uchar4>(colorTextureRef, p10.x, p10.y); if(v10.x != 0) { s0 +=		 alpha *make_float4(v10); w0 +=		 alpha ; } }

	float4 s1 = make_float4(0.0f, 0.0f, 0.0f, 0.0f); float w1 = 0.0f;
	if(p01.x >= 0 && p01.x < imageWidth && p01.y >= 0 && p01.y < imageHeight) { uchar4 v01 = tex2D<uchar4>(colorTextureRef, p01.x, p01.y); if(v01.x != 0) { s1 += (1.0f-alpha)*make_float4(v01); w1 += (1.0f-alpha);} }
	if(p11.x >= 0 && p11.x < imageWidth && p11.y >= 0 && p11.y < imageHeight) { uchar4 v11 = tex2D<uchar4>(colorTextureRef, p11.x, p11.y); if(v11.x != 0) { s1 +=		 alpha *make_float4(v11); w1 +=		 alpha ;} }

	const float4 p0 = s0/w0;
	const float4 p1 = s1/w1;

	float4 ss = make_float4(0.0f, 0.0f, 0.0f, 0.0f); float ww = 0.0f;
	if(w0 > 0.0f) { ss += (1.0f-beta)*p0; ww += (1.0f-beta); }
	if(w1 > 0.0f) { ss +=		beta *p1; ww +=		  beta ; }

	if(ww > 0.0f) {
		ss /= ww;
		return make_uchar4(ss.x,ss.y,ss.z,ss.w);
	} else		  return make_uchar4(0, 0, 0, 0);
}

__device__ float colourDistance(const uchar4 &c1, const uchar3 &c2) {
	float x = c1.x-c2.x;
	float y = c1.y-c2.y;
	float z = c1.z-c2.z;
	return x*x + y*y + z*z;
}

__device__ bool colordiff(const uchar4 &pa, const uchar3 &pb, float epsilon) {
	float x_2 = pb.x * pb.x + pb.y * pb.y + pb.z * pb.z;
	float v_2 = pa.x * pa.x + pa.y * pa.y + pa.z * pa.z;
	float xv_2 = pow(pb.x * pa.x + pb.y * pa.y + pb.z * pa.z, 2);
	float p_2 = xv_2 / v_2;
	return sqrt(x_2 - p_2) < epsilon;
}

#define NUM_CUDA_BLOCKS		10000



/*__global__ void integrateRegistrationKernel(HashData hashData, HashParams hashParams, DepthCameraParams cameraParams, cudaTextureObject_t depthT, cudaTextureObject_t colourT) {

	// Stride over all allocated blocks
	for (int bi=blockIdx.x; bi<*hashData.d_hashCompactifiedCounter; bi+=NUM_CUDA_BLOCKS) {

	//TODO check if we should load this in shared memory
	HashEntry& entry = hashData.d_hashCompactified[bi];


	int3 pi_base = hashData.SDFBlockToVirtualVoxelPos(entry.pos);

	uint i = threadIdx.x;	//inside of an SDF block
	int3 pi = pi_base + make_int3(hashData.delinearizeVoxelIndex(i));
	float3 pf = hashData.virtualVoxelPosToWorld(pi);

	pf = hashParams.m_rigidTransformInverse * pf;
	uint2 screenPos = make_uint2(cameraParams.cameraToKinectScreenInt(pf));

	// For this voxel in hash, get its screen position and check it is on screen
	if (screenPos.x < cameraParams.m_imageWidth && screenPos.y < cameraParams.m_imageHeight) {	//on screen

		//float depth = g_InputDepth[screenPos];
		float depth = tex2D<float>(depthT, screenPos.x, screenPos.y);
		//if (depth > 20.0f) return;

		uchar4 color  = make_uchar4(0, 0, 0, 0);
		color = tex2D<uchar4>(colourT, screenPos.x, screenPos.y);

		// Depth is within accepted max distance from camera
		if (depth > 0.01f && depth < hashParams.m_maxIntegrationDistance) { // valid depth and color (Nick: removed colour check)
			float depthZeroOne = cameraParams.cameraToKinectProjZ(depth);

			// Calculate SDF of this voxel wrt the depth map value
			float sdf = depth - pf.z;
			float truncation = hashData.getTruncation(depth);

			if (sdf > -truncation) {
				float weightUpdate = max(hashParams.m_integrationWeightSample * 1.5f * (1.0f-depthZeroOne), 1.0f);

				Voxel curr;	//construct current voxel
				curr.sdf = sdf;
				curr.weight = weightUpdate;
				curr.color = make_uchar3(color.x, color.y, color.z);

				uint idx = entry.ptr + i;
				
				Voxel out;
				const Voxel &v1 = curr;
				const Voxel &v0 = hashData.d_SDFBlocks[idx];

				float redshift = (v0.weight > 0) ? 1.0f - ((v1.sdf - v0.sdf) / hashParams.m_truncation)*0.5f : 1.0f;

				out.color.x = min(max(v1.color.x*redshift,0.0f),255.0f);
				out.color.y = min(max(v1.color.y*redshift,0.0f),255.0f);
				out.color.z = min(max(v1.color.z*(1.0f / redshift),0.0f),255.0f);

				out.sdf = (v0.sdf * (float)v0.weight + v1.sdf * (float)v1.weight) / ((float)v0.weight + (float)v1.weight);
				out.weight = min(c_hashParams.m_integrationWeightMax, (unsigned int)v0.weight + (unsigned int)v1.weight);

				hashData.d_SDFBlocks[idx] = out;

			}
		}
	}

	}  // Stride loop
}*/

extern __constant__ ftl::voxhash::DepthCameraCUDA c_cameras[MAX_CAMERAS];

#define WARP_SIZE 32

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
	uint ballot_result = __ballot_sync(0xFFFFFFFF, (oldVoxel.sdf >= 0.0f) ? 0 : 1);

	// Aggregate each warp result into voxel mask
	if (i % WARP_SIZE == 0) {
		voxels[warpNum] = ballot_result;
	}

	__syncthreads();

	// Work out if block is occupied or not and save voxel masks
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



void ftl::cuda::integrateDepthMaps(HashData& hashData, const HashParams& hashParams, int numcams, cudaStream_t stream) {
const unsigned int threadsPerBlock = SDF_BLOCK_SIZE*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE;
const dim3 gridSize(NUM_CUDA_BLOCKS, 1);
const dim3 blockSize(threadsPerBlock, 1);

//if (hashParams.m_numOccupiedBlocks > 0) {	//this guard is important if there is no depth in the current frame (i.e., no blocks were allocated)
	integrateDepthMapsKernel << <gridSize, blockSize, 0, stream >> >(hashData, hashParams, numcams);
//}

//cudaSafeCall( cudaGetLastError() );
#ifdef _DEBUG
cudaSafeCall(cudaDeviceSynchronize());
//cutilCheckMsg(__FUNCTION__);
#endif
}
