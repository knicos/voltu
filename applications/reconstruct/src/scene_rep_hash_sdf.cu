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
__device__ __constant__ ftl::voxhash::DepthCameraCUDA c_cameras[MAX_CAMERAS];

extern "C" void updateConstantHashParams(const HashParams& params) {

	size_t size;
	cudaSafeCall(cudaGetSymbolSize(&size, c_hashParams));
	cudaSafeCall(cudaMemcpyToSymbol(c_hashParams, &params, size, 0, cudaMemcpyHostToDevice));
	
#ifdef DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
#endif
	}


/*extern "C" void updateConstantRayCastParams(const RayCastParams& params) {
	//printf("Update ray cast params\n");
	size_t size;
	cudaSafeCall(cudaGetSymbolSize(&size, c_rayCastParams));
	cudaSafeCall(cudaMemcpyToSymbol(c_rayCastParams, &params, size, 0, cudaMemcpyHostToDevice));
	
#ifdef DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
#endif

}*/

extern "C" void updateCUDACameraConstant(ftl::voxhash::DepthCameraCUDA *data, int count) {
	if (count == 0 || count >= MAX_CAMERAS) return;
	//printf("Update depth camera params\n");
	size_t size;
	cudaSafeCall(cudaGetSymbolSize(&size, c_cameras));
	cudaSafeCall(cudaMemcpyToSymbol(c_cameras, data, sizeof(ftl::voxhash::DepthCameraCUDA)*count, 0, cudaMemcpyHostToDevice));
	
#ifdef DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
#endif

}

/*__global__ void resetHeapKernel(HashData hashData) 
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
}*/

__global__ void resetHashKernel(HashData hashData) 
{
	const HashParams& hashParams = c_hashParams;
	const unsigned int idx = blockIdx.x*blockDim.x + threadIdx.x;
	if (idx < hashParams.m_hashNumBuckets) {
		hashData.deleteHashEntry(hashData.d_hash[idx]);
		//hashData.deleteHashEntry(hashData.d_hashCompactified[idx]);
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
	//{
		//resetting the heap and SDF blocks
		//const dim3 gridSize((hashParams.m_numSDFBlocks + (T_PER_BLOCK*T_PER_BLOCK) - 1)/(T_PER_BLOCK*T_PER_BLOCK), 1);
		//const dim3 blockSize((T_PER_BLOCK*T_PER_BLOCK), 1);

		//resetHeapKernel<<<gridSize, blockSize>>>(hashData);


		//#ifdef _DEBUG
		//	cudaSafeCall(cudaDeviceSynchronize());
			//cutilCheckMsg(__FUNCTION__);
		//#endif

	//}

	{
		//resetting the hash
		const dim3 gridSize((hashParams.m_hashNumBuckets + (T_PER_BLOCK*T_PER_BLOCK) - 1)/(T_PER_BLOCK*T_PER_BLOCK), 1);
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


// Note: bitMask used for Streaming out code... could be set to nullptr if not streaming out
// Note: Allocations might need to be around fat rays since multiple voxels could correspond
// to same depth map pixel at larger distances.
__global__ void allocKernel(HashData hashData, HashParams hashParams, int camnum) 
{
	//const HashParams& hashParams = c_hashParams;
	const ftl::voxhash::DepthCameraCUDA camera = c_cameras[camnum];
	const DepthCameraParams &cameraParams = camera.params;

	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < cameraParams.m_imageWidth && y < cameraParams.m_imageHeight)
	{
		float d = tex2D<float>(camera.depth, x, y);
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
		rayMin = camera.pose * rayMin;
		//printf("Ray min: %f,%f,%f\n", rayMin.x, rayMin.y, rayMin.z);
		float3 rayMax = cameraParams.kinectDepthToSkeleton(x, y, maxDepth);
		rayMax = camera.pose * rayMax;

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
			if (hashData.isInBoundingBox(hashParams, idCurrentVoxel)) { //} && !isSDFBlockStreamedOut(idCurrentVoxel, hashData, d_bitMask)) {		
				hashData.allocBlock(idCurrentVoxel);
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

extern "C" void allocCUDA(HashData& hashData, const HashParams& hashParams, int camnum, const DepthCameraParams &depthCameraParams, cudaStream_t stream) 
{

	//printf("Allocating: %d\n",depthCameraParams.m_imageWidth);

	const dim3 gridSize((depthCameraParams.m_imageWidth + T_PER_BLOCK - 1)/T_PER_BLOCK, (depthCameraParams.m_imageHeight + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	allocKernel<<<gridSize, blockSize, 0, stream>>>(hashData, hashParams, camnum);


	#ifdef _DEBUG
		cudaSafeCall(cudaDeviceSynchronize());
		//cutilCheckMsg(__FUNCTION__);
	#endif
}
