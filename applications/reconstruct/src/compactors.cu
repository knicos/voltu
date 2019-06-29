#include "compactors.hpp"

using ftl::voxhash::HashData;
using ftl::voxhash::HashParams;
using ftl::voxhash::Voxel;
using ftl::voxhash::HashEntry;
using ftl::voxhash::FREE_ENTRY;

#define COMPACTIFY_HASH_THREADS_PER_BLOCK 256
//#define COMPACTIFY_HASH_SIMPLE


/*__global__ void fillDecisionArrayKernel(HashData hashData, DepthCameraData depthCameraData) 
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
}*/

/*extern "C" void fillDecisionArrayCUDA(HashData& hashData, const HashParams& hashParams, const DepthCameraData& depthCameraData)
{
	const dim3 gridSize((HASH_BUCKET_SIZE * hashParams.m_hashNumBuckets + (T_PER_BLOCK*T_PER_BLOCK) - 1)/(T_PER_BLOCK*T_PER_BLOCK), 1);
	const dim3 blockSize((T_PER_BLOCK*T_PER_BLOCK), 1);

	fillDecisionArrayKernel<<<gridSize, blockSize>>>(hashData, depthCameraData);

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
#endif

}*/

/*__global__ void compactifyHashKernel(HashData hashData) 
{
	const HashParams& hashParams = c_hashParams;
	const unsigned int idx = blockIdx.x*blockDim.x + threadIdx.x;
	if (idx < hashParams.m_hashNumBuckets * HASH_BUCKET_SIZE) {
		if (hashData.d_hashDecision[idx] == 1) {
			hashData.d_hashCompactified[hashData.d_hashDecisionPrefix[idx]-1] = hashData.d_hash[idx];
		}
	}
}*/

/*extern "C" void compactifyHashCUDA(HashData& hashData, const HashParams& hashParams) 
{
	const dim3 gridSize((HASH_BUCKET_SIZE * hashParams.m_hashNumBuckets + (T_PER_BLOCK*T_PER_BLOCK) - 1)/(T_PER_BLOCK*T_PER_BLOCK), 1);
	const dim3 blockSize((T_PER_BLOCK*T_PER_BLOCK), 1);

	compactifyHashKernel<<<gridSize, blockSize>>>(hashData);

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
#endif
}*/

__global__ void compactifyVisibleKernel(HashData hashData, HashParams hashParams, DepthCameraParams camera)
{
	//const HashParams& hashParams = c_hashParams;
	const unsigned int idx = blockIdx.x*blockDim.x + threadIdx.x;
#ifdef COMPACTIFY_HASH_SIMPLE
	if (idx < hashParams.m_hashNumBuckets * HASH_BUCKET_SIZE) {
		if (hashData.d_hash[idx].ptr != FREE_ENTRY) {
			if (hashData.isSDFBlockInCameraFrustumApprox(hashParams, camera, hashData.d_hash[idx].pos))
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
			if (hashData.isSDFBlockInCameraFrustumApprox(hashParams, camera, hashData.d_hash[idx].pos))
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

void ftl::cuda::compactifyVisible(HashData& hashData, const HashParams& hashParams, const DepthCameraParams &camera, cudaStream_t stream) {
	const unsigned int threadsPerBlock = COMPACTIFY_HASH_THREADS_PER_BLOCK;
	const dim3 gridSize((HASH_BUCKET_SIZE * hashParams.m_hashNumBuckets + threadsPerBlock - 1) / threadsPerBlock, 1);
	const dim3 blockSize(threadsPerBlock, 1);

	cudaSafeCall(cudaMemsetAsync(hashData.d_hashCompactifiedCounter, 0, sizeof(int),stream));
	compactifyVisibleKernel << <gridSize, blockSize, 0, stream >> >(hashData, hashParams, camera);
	//unsigned int res = 0;
	//cudaSafeCall(cudaMemcpyAsync(&res, hashData.d_hashCompactifiedCounter, sizeof(unsigned int), cudaMemcpyDeviceToHost, stream));

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
#endif
	//return res;
}

__global__ void compactifyAllocatedKernel(HashData hashData)
{
	const HashParams& hashParams = c_hashParams;
	const unsigned int idx = blockIdx.x*blockDim.x + threadIdx.x;
#ifdef COMPACTIFY_HASH_SIMPLE
	if (idx < hashParams.m_hashNumBuckets * HASH_BUCKET_SIZE) {
		if (hashData.d_hash[idx].ptr != FREE_ENTRY) {
			int addr = atomicAdd(hashData.d_hashCompactifiedCounter, 1);
			hashData.d_hashCompactified[addr] = hashData.d_hash[idx];
		}
	}
#else	
	__shared__ int localCounter;
	if (threadIdx.x == 0) localCounter = 0;
	__syncthreads();

	int addrLocal = -1;
	if (idx < hashParams.m_hashNumBuckets * HASH_BUCKET_SIZE) {
		if (hashData.d_hash[idx].ptr != FREE_ENTRY) {
			addrLocal = atomicAdd(&localCounter, 1);
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

void ftl::cuda::compactifyAllocated(HashData& hashData, const HashParams& hashParams, cudaStream_t stream) {
	const unsigned int threadsPerBlock = COMPACTIFY_HASH_THREADS_PER_BLOCK;
	const dim3 gridSize((HASH_BUCKET_SIZE * hashParams.m_hashNumBuckets + threadsPerBlock - 1) / threadsPerBlock, 1);
	const dim3 blockSize(threadsPerBlock, 1);

	cudaSafeCall(cudaMemsetAsync(hashData.d_hashCompactifiedCounter, 0, sizeof(int), stream));
	compactifyAllocatedKernel << <gridSize, blockSize, 0, stream >> >(hashData);
	//unsigned int res = 0;
	//cudaSafeCall(cudaMemcpyAsync(&res, hashData.d_hashCompactifiedCounter, sizeof(unsigned int), cudaMemcpyDeviceToHost, stream));

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
#endif
	//return res;
}
