// From: https://github.com/niessner/VoxelHashing/blob/master/DepthSensingCUDA/Source/CUDASceneRepHashSDF.cu

//#include <cutil_inline.h>
//#include <cutil_math.h>
#include <vector_types.h>
#include <cuda_runtime.h>

#include <ftl/cuda_matrix_util.hpp>

#include <ftl/voxel_hash.hpp>
#include <ftl/depth_camera.hpp>
#include <ftl/ray_cast_params.hpp>

#define T_PER_BLOCK 8

using ftl::voxhash::HashData;
using ftl::voxhash::HashParams;
using ftl::voxhash::Voxel;
using ftl::voxhash::HashEntry;
using ftl::voxhash::FREE_ENTRY;

// TODO (Nick) Use ftl::cuda::Texture (texture objects)
//texture<float, cudaTextureType2D, cudaReadModeElementType> depthTextureRef;
//texture<float4, cudaTextureType2D, cudaReadModeElementType> colorTextureRef;

__device__ __constant__ HashParams c_hashParams;
__device__ __constant__ RayCastParams c_rayCastParams;
__device__ __constant__ DepthCameraParams c_depthCameraParams;

extern "C" void updateConstantHashParams(const HashParams& params) {

	size_t size;
	cudaSafeCall(cudaGetSymbolSize(&size, c_hashParams));
	cudaSafeCall(cudaMemcpyToSymbol(c_hashParams, &params, size, 0, cudaMemcpyHostToDevice));
	
#ifdef DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
#endif
	}


extern "C" void updateConstantRayCastParams(const RayCastParams& params) {
	//printf("Update ray cast params\n");
	size_t size;
	cudaSafeCall(cudaGetSymbolSize(&size, c_rayCastParams));
	cudaSafeCall(cudaMemcpyToSymbol(c_rayCastParams, &params, size, 0, cudaMemcpyHostToDevice));
	
#ifdef DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
#endif

}

extern "C" void updateConstantDepthCameraParams(const DepthCameraParams& params) {
	//printf("Update depth camera params\n");
	size_t size;
	cudaSafeCall(cudaGetSymbolSize(&size, c_depthCameraParams));
	cudaSafeCall(cudaMemcpyToSymbol(c_depthCameraParams, &params, size, 0, cudaMemcpyHostToDevice));
	
#ifdef DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
#endif

}

extern "C" void bindInputDepthColorTextures(const DepthCameraData& depthCameraData) 
{
	/*cudaSafeCall(cudaBindTextureToArray(depthTextureRef, depthCameraData.d_depthArray, depthCameraData.h_depthChannelDesc));
	cudaSafeCall(cudaBindTextureToArray(colorTextureRef, depthCameraData.d_colorArray, depthCameraData.h_colorChannelDesc));
	depthTextureRef.filterMode = cudaFilterModePoint;
	colorTextureRef.filterMode = cudaFilterModePoint;*/
}

__global__ void resetHeapKernel(HashData hashData) 
{
	const HashParams& hashParams = c_hashParams;
	unsigned int idx = blockIdx.x*blockDim.x + threadIdx.x;

	if (idx == 0) {
		hashData.d_heapCounter[0] = hashParams.m_numSDFBlocks - 1;	//points to the last element of the array
	}
	
	if (idx < hashParams.m_numSDFBlocks) {

		hashData.d_heap[idx] = hashParams.m_numSDFBlocks - idx - 1;
		uint blockSize = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
		uint base_idx = idx * blockSize;
		for (uint i = 0; i < blockSize; i++) {
			hashData.deleteVoxel(base_idx+i);
		}
	}
}

__global__ void resetHashKernel(HashData hashData) 
{
	const HashParams& hashParams = c_hashParams;
	const unsigned int idx = blockIdx.x*blockDim.x + threadIdx.x;
	if (idx < hashParams.m_hashNumBuckets * HASH_BUCKET_SIZE) {
		hashData.deleteHashEntry(hashData.d_hash[idx]);
		hashData.deleteHashEntry(hashData.d_hashCompactified[idx]);
	}
}


__global__ void resetHashBucketMutexKernel(HashData hashData) 
{
	const HashParams& hashParams = c_hashParams;
	const unsigned int idx = blockIdx.x*blockDim.x + threadIdx.x;
	if (idx < hashParams.m_hashNumBuckets) {
		hashData.d_hashBucketMutex[idx] = FREE_ENTRY;
	}
}

extern "C" void resetCUDA(HashData& hashData, const HashParams& hashParams)
{
	{
		//resetting the heap and SDF blocks
		const dim3 gridSize((hashParams.m_numSDFBlocks + (T_PER_BLOCK*T_PER_BLOCK) - 1)/(T_PER_BLOCK*T_PER_BLOCK), 1);
		const dim3 blockSize((T_PER_BLOCK*T_PER_BLOCK), 1);

		resetHeapKernel<<<gridSize, blockSize>>>(hashData);


		#ifdef _DEBUG
			cudaSafeCall(cudaDeviceSynchronize());
			//cutilCheckMsg(__FUNCTION__);
		#endif

	}

	{
		//resetting the hash
		const dim3 gridSize((HASH_BUCKET_SIZE * hashParams.m_hashNumBuckets + (T_PER_BLOCK*T_PER_BLOCK) - 1)/(T_PER_BLOCK*T_PER_BLOCK), 1);
		const dim3 blockSize((T_PER_BLOCK*T_PER_BLOCK), 1);

		resetHashKernel<<<gridSize, blockSize>>>(hashData);

		#ifdef _DEBUG
			cudaSafeCall(cudaDeviceSynchronize());
			//cutilCheckMsg(__FUNCTION__);
		#endif
	}

	{
		//resetting the mutex
		const dim3 gridSize((hashParams.m_hashNumBuckets + (T_PER_BLOCK*T_PER_BLOCK) - 1)/(T_PER_BLOCK*T_PER_BLOCK), 1);
		const dim3 blockSize((T_PER_BLOCK*T_PER_BLOCK), 1);

		resetHashBucketMutexKernel<<<gridSize, blockSize>>>(hashData);

		#ifdef _DEBUG
			cudaSafeCall(cudaDeviceSynchronize());
			//cutilCheckMsg(__FUNCTION__);
		#endif
	}


}

extern "C" void resetHashBucketMutexCUDA(HashData& hashData, const HashParams& hashParams)
{
	const dim3 gridSize((hashParams.m_hashNumBuckets + (T_PER_BLOCK*T_PER_BLOCK) - 1)/(T_PER_BLOCK*T_PER_BLOCK), 1);
	const dim3 blockSize((T_PER_BLOCK*T_PER_BLOCK), 1);

	resetHashBucketMutexKernel<<<gridSize, blockSize>>>(hashData);

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
#endif
}


__device__
unsigned int linearizeChunkPos(const int3& chunkPos)
{
	int3 p = chunkPos-c_hashParams.m_streamingMinGridPos;
	return  p.z * c_hashParams.m_streamingGridDimensions.x * c_hashParams.m_streamingGridDimensions.y +
			p.y * c_hashParams.m_streamingGridDimensions.x +
			p.x;
}

__device__
int3 worldToChunks(const float3& posWorld)
{
	float3 p;
	p.x = posWorld.x/c_hashParams.m_streamingVoxelExtents.x;
	p.y = posWorld.y/c_hashParams.m_streamingVoxelExtents.y;
	p.z = posWorld.z/c_hashParams.m_streamingVoxelExtents.z;

	float3 s;
	s.x = (float)sign(p.x);
	s.y = (float)sign(p.y);
	s.z = (float)sign(p.z);

	return make_int3(p+s*0.5f);
}

__device__
bool isSDFBlockStreamedOut(const int3& sdfBlock, const HashData& hashData, const unsigned int* d_bitMask)	//TODO MATTHIAS (-> move to HashData)
{
	float3 posWorld = hashData.virtualVoxelPosToWorld(hashData.SDFBlockToVirtualVoxelPos(sdfBlock)); // sdfBlock is assigned to chunk by the bottom right sample pos

	uint index = linearizeChunkPos(worldToChunks(posWorld));
	uint nBitsInT = 32;
	return ((d_bitMask[index/nBitsInT] & (0x1 << (index%nBitsInT))) != 0x0);
}

// Note: bitMask used for Streaming out code... could be set to nullptr if not streaming out
// Note: Allocations might need to be around fat rays since multiple voxels could correspond
// to same depth map pixel at larger distances.
__global__ void allocKernel(HashData hashData, DepthCameraData cameraData, const unsigned int* d_bitMask) 
{
	const HashParams& hashParams = c_hashParams;
	const DepthCameraParams& cameraParams = c_depthCameraParams;

	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < cameraParams.m_imageWidth && y < cameraParams.m_imageHeight)
	{
		float d = tex2D<float>(cameraData.depth_obj_, x, y);
		//if (d == MINF || d < cameraParams.m_sensorDepthWorldMin || d > cameraParams.m_sensorDepthWorldMax)	return;
		if (d == MINF || d == 0.0f)	return;

		if (d >= hashParams.m_maxIntegrationDistance) return;

		// TODO (Nick) Use covariance to include a frustrum of influence
		float t = hashData.getTruncation(d);
		float minDepth = min(hashParams.m_maxIntegrationDistance, d-t);
		float maxDepth = min(hashParams.m_maxIntegrationDistance, d+t);
		if (minDepth >= maxDepth) return;

		// Convert ray from image coords to world
		// Does kinectDepthToSkeleton convert pixel values to coordinates using
		// camera intrinsics? Same as what reprojectTo3D does in OpenCV?
		float3 rayMin = cameraData.kinectDepthToSkeleton(x, y, minDepth);
		// Is the rigid transform then the estimated camera pose?
		rayMin = hashParams.m_rigidTransform * rayMin;
		//printf("Ray min: %f,%f,%f\n", rayMin.x, rayMin.y, rayMin.z);
		float3 rayMax = cameraData.kinectDepthToSkeleton(x, y, maxDepth);
		rayMax = hashParams.m_rigidTransform * rayMax;

		float3 rayDir = normalize(rayMax - rayMin);
	
		// Only ray cast from min possible depth to max depth
		int3 idCurrentVoxel = hashData.worldToSDFBlock(rayMin);
		int3 idEnd = hashData.worldToSDFBlock(rayMax);
		
		float3 step = make_float3(sign(rayDir));
		float3 boundaryPos = hashData.SDFBlockToWorld(idCurrentVoxel+make_int3(clamp(step, 0.0, 1.0f)))-0.5f*hashParams.m_virtualVoxelSize;
		float3 tMax = (boundaryPos-rayMin)/rayDir;
		float3 tDelta = (step*SDF_BLOCK_SIZE*hashParams.m_virtualVoxelSize)/rayDir;
		int3 idBound = make_int3(make_float3(idEnd)+step);

		//#pragma unroll
		//for(int c = 0; c < 3; c++) {
		//	if (rayDir[c] == 0.0f) { tMax[c] = PINF; tDelta[c] = PINF; }
		//	if (boundaryPos[c] - rayMin[c] == 0.0f) { tMax[c] = PINF; tDelta[c] = PINF; }
		//}
		if (rayDir.x == 0.0f) { tMax.x = PINF; tDelta.x = PINF; }
		if (boundaryPos.x - rayMin.x == 0.0f) { tMax.x = PINF; tDelta.x = PINF; }

		if (rayDir.y == 0.0f) { tMax.y = PINF; tDelta.y = PINF; }
		if (boundaryPos.y - rayMin.y == 0.0f) { tMax.y = PINF; tDelta.y = PINF; }

		if (rayDir.z == 0.0f) { tMax.z = PINF; tDelta.z = PINF; }
		if (boundaryPos.z - rayMin.z == 0.0f) { tMax.z = PINF; tDelta.z = PINF; }


		unsigned int iter = 0; // iter < g_MaxLoopIterCount
		unsigned int g_MaxLoopIterCount = 1024;
#pragma unroll 1
		while(iter < g_MaxLoopIterCount) {

			//check if it's in the frustum and not checked out
			if (hashData.isSDFBlockInCameraFrustumApprox(idCurrentVoxel)) { //} && !isSDFBlockStreamedOut(idCurrentVoxel, hashData, d_bitMask)) {		
				hashData.allocBlock(idCurrentVoxel, cameraParams.flags & 0xFF);
				//printf("Allocate block: %d\n",idCurrentVoxel.x);
			}

			// Traverse voxel grid
			if(tMax.x < tMax.y && tMax.x < tMax.z)	{
				idCurrentVoxel.x += step.x;
				if(idCurrentVoxel.x == idBound.x) return;
				tMax.x += tDelta.x;
			}
			else if(tMax.z < tMax.y) {
				idCurrentVoxel.z += step.z;
				if(idCurrentVoxel.z == idBound.z) return;
				tMax.z += tDelta.z;
			}
			else	{
				idCurrentVoxel.y += step.y;
				if(idCurrentVoxel.y == idBound.y) return;
				tMax.y += tDelta.y;
			}

			iter++;
		}
	}
}

extern "C" void allocCUDA(HashData& hashData, const HashParams& hashParams, const DepthCameraData& depthCameraData, const DepthCameraParams& depthCameraParams, const unsigned int* d_bitMask) 
{

	//printf("Allocating: %d\n",depthCameraParams.m_imageWidth);

	const dim3 gridSize((depthCameraParams.m_imageWidth + T_PER_BLOCK - 1)/T_PER_BLOCK, (depthCameraParams.m_imageHeight + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	allocKernel<<<gridSize, blockSize>>>(hashData, depthCameraData, d_bitMask);

	//cudaSafeCall(cudaDeviceSynchronize());

	#ifdef _DEBUG
		cudaSafeCall(cudaDeviceSynchronize());
		//cutilCheckMsg(__FUNCTION__);
	#endif
}



__global__ void fillDecisionArrayKernel(HashData hashData, DepthCameraData depthCameraData) 
{
	const HashParams& hashParams = c_hashParams;
	const unsigned int idx = blockIdx.x*blockDim.x + threadIdx.x;

	if (idx < hashParams.m_hashNumBuckets * HASH_BUCKET_SIZE) {
		hashData.d_hashDecision[idx] = 0;
		if (hashData.d_hash[idx].ptr != FREE_ENTRY) {
			if (hashData.isSDFBlockInCameraFrustumApprox(hashData.d_hash[idx].pos)) {
				hashData.d_hashDecision[idx] = 1;	//yes
			}
		}
	}
}

extern "C" void fillDecisionArrayCUDA(HashData& hashData, const HashParams& hashParams, const DepthCameraData& depthCameraData)
{
	const dim3 gridSize((HASH_BUCKET_SIZE * hashParams.m_hashNumBuckets + (T_PER_BLOCK*T_PER_BLOCK) - 1)/(T_PER_BLOCK*T_PER_BLOCK), 1);
	const dim3 blockSize((T_PER_BLOCK*T_PER_BLOCK), 1);

	fillDecisionArrayKernel<<<gridSize, blockSize>>>(hashData, depthCameraData);

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
#endif

}

__global__ void compactifyHashKernel(HashData hashData) 
{
	const HashParams& hashParams = c_hashParams;
	const unsigned int idx = blockIdx.x*blockDim.x + threadIdx.x;
	if (idx < hashParams.m_hashNumBuckets * HASH_BUCKET_SIZE) {
		if (hashData.d_hashDecision[idx] == 1) {
			hashData.d_hashCompactified[hashData.d_hashDecisionPrefix[idx]-1] = hashData.d_hash[idx];
		}
	}
}

extern "C" void compactifyHashCUDA(HashData& hashData, const HashParams& hashParams) 
{
	const dim3 gridSize((HASH_BUCKET_SIZE * hashParams.m_hashNumBuckets + (T_PER_BLOCK*T_PER_BLOCK) - 1)/(T_PER_BLOCK*T_PER_BLOCK), 1);
	const dim3 blockSize((T_PER_BLOCK*T_PER_BLOCK), 1);

	compactifyHashKernel<<<gridSize, blockSize>>>(hashData);

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
#endif
}


#define COMPACTIFY_HASH_THREADS_PER_BLOCK 256
//#define COMPACTIFY_HASH_SIMPLE
__global__ void compactifyHashAllInOneKernel(HashData hashData)
{
	const HashParams& hashParams = c_hashParams;
	const unsigned int idx = blockIdx.x*blockDim.x + threadIdx.x;
#ifdef COMPACTIFY_HASH_SIMPLE
	if (idx < hashParams.m_hashNumBuckets * HASH_BUCKET_SIZE) {
		if (hashData.d_hash[idx].ptr != FREE_ENTRY) {
			if (hashData.isSDFBlockInCameraFrustumApprox(hashData.d_hash[idx].pos))
			{
				int addr = atomicAdd(hashData.d_hashCompactifiedCounter, 1);
				hashData.d_hashCompactified[addr] = hashData.d_hash[idx];
			}
		}
	}
#else	
	__shared__ int localCounter;
	if (threadIdx.x == 0) localCounter = 0;
	__syncthreads();

	int addrLocal = -1;
	if (idx < hashParams.m_hashNumBuckets * HASH_BUCKET_SIZE) {
		if (hashData.d_hash[idx].ptr != FREE_ENTRY) {
			if (hashData.isSDFBlockInCameraFrustumApprox(hashData.d_hash[idx].pos))
			{
				addrLocal = atomicAdd(&localCounter, 1);
			}
		}
	}

	__syncthreads();

	__shared__ int addrGlobal;
	if (threadIdx.x == 0 && localCounter > 0) {
		addrGlobal = atomicAdd(hashData.d_hashCompactifiedCounter, localCounter);
	}
	__syncthreads();

	if (addrLocal != -1) {
		const unsigned int addr = addrGlobal + addrLocal;
		hashData.d_hashCompactified[addr] = hashData.d_hash[idx];
	}
#endif
}

extern "C" unsigned int compactifyHashAllInOneCUDA(HashData& hashData, const HashParams& hashParams)
{
	const unsigned int threadsPerBlock = COMPACTIFY_HASH_THREADS_PER_BLOCK;
	const dim3 gridSize((HASH_BUCKET_SIZE * hashParams.m_hashNumBuckets + threadsPerBlock - 1) / threadsPerBlock, 1);
	const dim3 blockSize(threadsPerBlock, 1);

	cudaSafeCall(cudaMemset(hashData.d_hashCompactifiedCounter, 0, sizeof(int)));
	compactifyHashAllInOneKernel << <gridSize, blockSize >> >(hashData);
	unsigned int res = 0;
	cudaSafeCall(cudaMemcpy(&res, hashData.d_hashCompactifiedCounter, sizeof(unsigned int), cudaMemcpyDeviceToHost));

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
#endif
	return res;
}

__device__ float4 make_float4(uchar4 c) {
	return make_float4(static_cast<float>(c.x), static_cast<float>(c.y), static_cast<float>(c.z), static_cast<float>(c.w));
}

inline __device__ uchar4 bilinearFilterColor(const float2& screenPos, cudaTextureObject_t colorTextureRef) {
	const DepthCameraParams& cameraParams = c_depthCameraParams;
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

__global__ void integrateDepthMapKernel(HashData hashData, DepthCameraData cameraData, cudaTextureObject_t depthT, cudaTextureObject_t colourT) {
	const HashParams& hashParams = c_hashParams;
	const DepthCameraParams& cameraParams = c_depthCameraParams;

	//TODO check if we should load this in shared memory
	HashEntry& entry = hashData.d_hashCompactified[blockIdx.x];
	//if (entry.ptr == FREE_ENTRY) {
	//	printf("invliad integrate");
	//	return; //should never happen since we did the compactification before
	//}

	int3 pi_base = hashData.SDFBlockToVirtualVoxelPos(entry.pos);

	uint i = threadIdx.x;	//inside of an SDF block
	int3 pi = pi_base + make_int3(hashData.delinearizeVoxelIndex(i));
	float3 pf = hashData.virtualVoxelPosToWorld(pi);

	pf = hashParams.m_rigidTransformInverse * pf;
	uint2 screenPos = make_uint2(cameraData.cameraToKinectScreenInt(pf));

	// For this voxel in hash, get its screen position and check it is on screen
	if (screenPos.x < cameraParams.m_imageWidth && screenPos.y < cameraParams.m_imageHeight) {	//on screen

		//float depth = g_InputDepth[screenPos];
		float depth = tex2D<float>(depthT, screenPos.x, screenPos.y);
		//if (depth > 20.0f) return;

		uchar4 color  = make_uchar4(0, 0, 0, 0);
		//if (cameraData.d_colorData) {
			color = tex2D<uchar4>(colourT, screenPos.x, screenPos.y);
			//color = bilinearFilterColor(cameraData.cameraToKinectScreenFloat(pf));
		//}

		//printf("screen pos %d\n", color.x);
		//return;

		// Depth is within accepted max distance from camera
		if (depth > 0 && depth < hashParams.m_maxIntegrationDistance) { // valid depth and color (Nick: removed colour check)
			float depthZeroOne = cameraData.cameraToKinectProjZ(depth);

			// Calculate SDF of this voxel wrt the depth map value
			float sdf = depth - pf.z;
			float truncation = hashData.getTruncation(depth);

			// Is this voxel close enough to cam for depth map value
			// CHECK Nick: If is too close then free space violation so remove?
			// This isn't enough if the disparity has occlusions that don't cause violations
			// Could RGB changes also cause removals if depth can't be confirmed?
			if (sdf > truncation) {
				uint idx = entry.ptr + i;
				hashData.d_SDFBlocks[idx].weight = 0;
				//hashData.d_SDFBlocks[idx].sdf = PINF;
				hashData.d_SDFBlocks[idx].color = make_uchar3(0,0,0);
			}
			else if (sdf > -truncation) // && depthZeroOne >= 0.0f && depthZeroOne <= 1.0f) //check if in truncation range should already be made in depth map computation
			{
				if (sdf >= 0.0f) {
					sdf = fminf(truncation, sdf);
				} else {
					sdf = fmaxf(-truncation, sdf);
				}


				//printf("SDF: %f\n", sdf);
				//float weightUpdate = g_WeightSample;
				//weightUpdate = (1-depthZeroOne)*5.0f + depthZeroOne*0.05f;
				//weightUpdate *= g_WeightSample;
				float weightUpdate = max(hashParams.m_integrationWeightSample * 1.5f * (1.0f-depthZeroOne), 1.0f);

				Voxel curr;	//construct current voxel
				curr.sdf = sdf;
				curr.weight = weightUpdate;
				curr.color = make_uchar3(color.x, color.y, color.z);
				

				uint idx = entry.ptr + i;

				if (entry.flags != cameraParams.flags & 0xFF) {
					entry.flags = cameraParams.flags & 0xFF;
					hashData.d_SDFBlocks[idx].color = make_uchar3(0,0,0);
				}
				
				Voxel newVoxel;
				//if (color.x == MINF) hashData.combineVoxelDepthOnly(hashData.d_SDFBlocks[idx], curr, newVoxel);
				//else hashData.combineVoxel(hashData.d_SDFBlocks[idx], curr, newVoxel);
				hashData.combineVoxel(hashData.d_SDFBlocks[idx], curr, newVoxel);

				hashData.d_SDFBlocks[idx] = newVoxel;

				//Voxel prev = getVoxel(g_SDFBlocksSDFUAV, g_SDFBlocksRGBWUAV, idx);
				//Voxel newVoxel = combineVoxel(curr, prev);
				//setVoxel(g_SDFBlocksSDFUAV, g_SDFBlocksRGBWUAV, idx, newVoxel);
			}
		} else {
			uint idx = entry.ptr + i;
			float coldist = colourDistance(color, hashData.d_SDFBlocks[idx].color);
			if (depth > 40.0f && coldist > 100.0f) {
				//hashData.d_SDFBlocks[idx].color = make_uchar3(0,0,(uchar)(coldist));
				hashData.d_SDFBlocks[idx].weight = hashData.d_SDFBlocks[idx].weight >> 1;
			}
		}
	}
}


extern "C" void integrateDepthMapCUDA(HashData& hashData, const HashParams& hashParams,
		const DepthCameraData& depthCameraData, const DepthCameraParams& depthCameraParams)
{
	const unsigned int threadsPerBlock = SDF_BLOCK_SIZE*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE;
	const dim3 gridSize(hashParams.m_numOccupiedBlocks, 1);
	const dim3 blockSize(threadsPerBlock, 1);

	if (hashParams.m_numOccupiedBlocks > 0) {	//this guard is important if there is no depth in the current frame (i.e., no blocks were allocated)
		integrateDepthMapKernel << <gridSize, blockSize >> >(hashData, depthCameraData, depthCameraData.depth_obj_, depthCameraData.colour_obj_);
	}

	cudaSafeCall( cudaGetLastError() );
#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
#endif
}



__global__ void starveVoxelsKernel(HashData hashData) {

	const uint idx = blockIdx.x;
	const HashEntry& entry = hashData.d_hashCompactified[idx];

	//is typically exectued only every n'th frame
	int weight = hashData.d_SDFBlocks[entry.ptr + threadIdx.x].weight;
	weight = max(0, weight-1);	
	hashData.d_SDFBlocks[entry.ptr + threadIdx.x].weight = weight;  //CHECK Remove to totally clear previous frame (Nick)
}

extern "C" void starveVoxelsKernelCUDA(HashData& hashData, const HashParams& hashParams)
{
	const unsigned int threadsPerBlock = SDF_BLOCK_SIZE*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE;
	const dim3 gridSize(hashParams.m_numOccupiedBlocks, 1);
	const dim3 blockSize(threadsPerBlock, 1);

	if (hashParams.m_numOccupiedBlocks > 0) {
		starveVoxelsKernel << <gridSize, blockSize >> >(hashData);
	}
#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
#endif
}


__shared__ float	shared_MinSDF[SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE / 2];
__shared__ uint		shared_MaxWeight[SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE / 2];


__global__ void garbageCollectIdentifyKernel(HashData hashData) {

	const DepthCameraParams& cameraParams = c_depthCameraParams;
	const unsigned int hashIdx = blockIdx.x;
	const HashEntry& entry = hashData.d_hashCompactified[hashIdx];

	// Entire block was not touched in this frame, so remove (Nick)
	/*if (entry.flags != cameraParams.flags & 0xFF) {
		hashData.d_hashDecision[hashIdx] = 1;
		return;
	}*/
	
	//uint h = hashData.computeHashPos(entry.pos);
	//hashData.d_hashDecision[hashIdx] = 1;
	//if (hashData.d_hashBucketMutex[h] == LOCK_ENTRY)	return;

	//if (entry.ptr == FREE_ENTRY) return; //should never happen since we did compactify before
	//const uint linBlockSize = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

	const unsigned int idx0 = entry.ptr + 2*threadIdx.x+0;
	const unsigned int idx1 = entry.ptr + 2*threadIdx.x+1;

	Voxel v0 = hashData.d_SDFBlocks[idx0];
	Voxel v1 = hashData.d_SDFBlocks[idx1];

	if (v0.weight == 0)	v0.sdf = PINF;
	if (v1.weight == 0)	v1.sdf = PINF;

	shared_MinSDF[threadIdx.x] = min(fabsf(v0.sdf), fabsf(v1.sdf));	//init shared memory
	shared_MaxWeight[threadIdx.x] = max(v0.weight, v1.weight);
		
#pragma unroll 1
	for (uint stride = 2; stride <= blockDim.x; stride <<= 1) {
		__syncthreads();
		if ((threadIdx.x  & (stride-1)) == (stride-1)) {
			shared_MinSDF[threadIdx.x] = min(shared_MinSDF[threadIdx.x-stride/2], shared_MinSDF[threadIdx.x]);
			shared_MaxWeight[threadIdx.x] = max(shared_MaxWeight[threadIdx.x-stride/2], shared_MaxWeight[threadIdx.x]);
		}
	}

	__syncthreads();

	if (threadIdx.x == blockDim.x - 1) {
		float minSDF = shared_MinSDF[threadIdx.x];
		uint maxWeight = shared_MaxWeight[threadIdx.x];

		float t = hashData.getTruncation(c_depthCameraParams.m_sensorDepthWorldMax);	//MATTHIAS TODO check whether this is a reasonable metric

		if (minSDF >= t || maxWeight == 0) {
			hashData.d_hashDecision[hashIdx] = 1;
		} else {
			hashData.d_hashDecision[hashIdx] = 0; 
		}
	}
}
 
extern "C" void garbageCollectIdentifyCUDA(HashData& hashData, const HashParams& hashParams) {
	
	const unsigned int threadsPerBlock = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE / 2;
	const dim3 gridSize(hashParams.m_numOccupiedBlocks, 1);
	const dim3 blockSize(threadsPerBlock, 1);

	if (hashParams.m_numOccupiedBlocks > 0) {
		garbageCollectIdentifyKernel << <gridSize, blockSize >> >(hashData);
	}
#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
#endif
}


__global__ void garbageCollectFreeKernel(HashData hashData) {

	//const uint hashIdx = blockIdx.x;
	const uint hashIdx = blockIdx.x*blockDim.x + threadIdx.x;


	if (hashIdx < c_hashParams.m_numOccupiedBlocks && hashData.d_hashDecision[hashIdx] != 0) {	//decision to delete the hash entry

		const HashEntry& entry = hashData.d_hashCompactified[hashIdx];
		//if (entry.ptr == FREE_ENTRY) return; //should never happen since we did compactify before

		if (hashData.deleteHashEntryElement(entry.pos)) {	//delete hash entry from hash (and performs heap append)
			const uint linBlockSize = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

			#pragma unroll 1
			for (uint i = 0; i < linBlockSize; i++) {	//clear sdf block: CHECK TODO another kernel?
				hashData.deleteVoxel(entry.ptr + i);
			}
		}
	}
}


extern "C" void garbageCollectFreeCUDA(HashData& hashData, const HashParams& hashParams) {
	
	const unsigned int threadsPerBlock = T_PER_BLOCK*T_PER_BLOCK;
	const dim3 gridSize((hashParams.m_numOccupiedBlocks + threadsPerBlock - 1) / threadsPerBlock, 1);
	const dim3 blockSize(threadsPerBlock, 1);
	
	if (hashParams.m_numOccupiedBlocks > 0) {
		garbageCollectFreeKernel << <gridSize, blockSize >> >(hashData);
	}
#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
#endif
}
