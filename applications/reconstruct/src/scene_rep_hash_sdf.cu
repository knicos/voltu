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
//__device__ __constant__ DepthCameraParams c_depthCameraParams;

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

/*extern "C" void updateConstantDepthCameraParams(const DepthCameraParams& params) {
	//printf("Update depth camera params\n");
	size_t size;
	cudaSafeCall(cudaGetSymbolSize(&size, c_depthCameraParams));
	cudaSafeCall(cudaMemcpyToSymbol(c_depthCameraParams, &params, size, 0, cudaMemcpyHostToDevice));
	
#ifdef DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
#endif

}*/

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

extern "C" void resetHashBucketMutexCUDA(HashData& hashData, const HashParams& hashParams, cudaStream_t stream)
{
	const dim3 gridSize((hashParams.m_hashNumBuckets + (T_PER_BLOCK*T_PER_BLOCK) - 1)/(T_PER_BLOCK*T_PER_BLOCK), 1);
	const dim3 blockSize((T_PER_BLOCK*T_PER_BLOCK), 1);

	resetHashBucketMutexKernel<<<gridSize, blockSize, 0, stream>>>(hashData);

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
__global__ void allocKernel(HashData hashData, DepthCameraData cameraData, HashParams hashParams, DepthCameraParams cameraParams) 
{
	//const HashParams& hashParams = c_hashParams;
	//const DepthCameraParams& cameraParams = c_depthCameraParams;

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
		float3 rayMin = cameraParams.kinectDepthToSkeleton(x, y, minDepth);
		// Is the rigid transform then the estimated camera pose?
		rayMin = hashParams.m_rigidTransform * rayMin;
		//printf("Ray min: %f,%f,%f\n", rayMin.x, rayMin.y, rayMin.z);
		float3 rayMax = cameraParams.kinectDepthToSkeleton(x, y, maxDepth);
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
			if (hashData.isSDFBlockInCameraFrustumApprox(hashParams, cameraParams, idCurrentVoxel)) { //} && !isSDFBlockStreamedOut(idCurrentVoxel, hashData, d_bitMask)) {		
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

extern "C" void allocCUDA(HashData& hashData, const HashParams& hashParams, const DepthCameraData& depthCameraData, const DepthCameraParams& depthCameraParams, cudaStream_t stream) 
{

	//printf("Allocating: %d\n",depthCameraParams.m_imageWidth);

	const dim3 gridSize((depthCameraParams.m_imageWidth + T_PER_BLOCK - 1)/T_PER_BLOCK, (depthCameraParams.m_imageHeight + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	allocKernel<<<gridSize, blockSize, 0, stream>>>(hashData, depthCameraData, hashParams, depthCameraParams);


	#ifdef _DEBUG
		cudaSafeCall(cudaDeviceSynchronize());
		//cutilCheckMsg(__FUNCTION__);
	#endif
}
