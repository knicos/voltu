#include <ftl/voxel_hash.hpp>
#include "garbage.hpp"

using namespace ftl::voxhash;

#define T_PER_BLOCK 8
#define NUM_CUDA_BLOCKS	10000

__global__ void starveVoxelsKernel(HashData hashData) {

	// Stride over all allocated blocks
	for (int bi=blockIdx.x; bi<*hashData.d_hashCompactifiedCounter; bi+=NUM_CUDA_BLOCKS) {

	const HashEntry& entry = hashData.d_hashCompactified[bi];

	//is typically exectued only every n'th frame
	int weight = hashData.d_SDFBlocks[entry.ptr + threadIdx.x].weight;
	weight = max(0, weight-2);	
	hashData.d_SDFBlocks[entry.ptr + threadIdx.x].weight = weight;  //CHECK Remove to totally clear previous frame (Nick)

	}
}

void ftl::cuda::starveVoxels(HashData& hashData, const HashParams& hashParams) {
	const unsigned int threadsPerBlock = SDF_BLOCK_SIZE*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE;
	const dim3 gridSize(NUM_CUDA_BLOCKS, 1);
	const dim3 blockSize(threadsPerBlock, 1);

	//if (hashParams.m_numOccupiedBlocks > 0) {
		starveVoxelsKernel << <gridSize, blockSize >> >(hashData);
	//}
#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
#endif
}


__shared__ float	shared_MinSDF[SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE / 2];
__shared__ uint		shared_MaxWeight[SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE / 2];


__global__ void garbageCollectIdentifyKernel(HashData hashData) {

	// Stride over all allocated blocks
	for (int bi=blockIdx.x; bi<*hashData.d_hashCompactifiedCounter; bi+=NUM_CUDA_BLOCKS) {

	const HashEntry& entry = hashData.d_hashCompactified[bi];

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

		float t = hashData.getTruncation(5.0f); // NICK should not be hardcoded	//MATTHIAS TODO check whether this is a reasonable metric

		if (minSDF >= t || maxWeight == 0) {
			hashData.d_hashDecision[bi] = 1;
		} else {
			hashData.d_hashDecision[bi] = 0; 
		}
	}

	}
}
 
void ftl::cuda::garbageCollectIdentify(HashData& hashData, const HashParams& hashParams, cudaStream_t stream) {
	
	const unsigned int threadsPerBlock = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE / 2;
	const dim3 gridSize(NUM_CUDA_BLOCKS, 1);
	const dim3 blockSize(threadsPerBlock, 1);

	//if (hashParams.m_numOccupiedBlocks > 0) {
		garbageCollectIdentifyKernel << <gridSize, blockSize, 0, stream >> >(hashData);
	//}
#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
#endif
}


__global__ void garbageCollectFreeKernel(HashData hashData) {

	// Stride over all allocated blocks
	for (int bi=blockIdx.x*blockDim.x + threadIdx.x; bi<*hashData.d_hashCompactifiedCounter; bi+=NUM_CUDA_BLOCKS) {

	if (hashData.d_hashDecision[bi] != 0) {	//decision to delete the hash entry

		const HashEntry& entry = hashData.d_hashCompactified[bi];
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
}


void ftl::cuda::garbageCollectFree(HashData& hashData, const HashParams& hashParams, cudaStream_t stream) {
	
	const unsigned int threadsPerBlock = T_PER_BLOCK*T_PER_BLOCK;
	const dim3 gridSize(NUM_CUDA_BLOCKS, 1);  // (hashParams.m_numOccupiedBlocks + threadsPerBlock - 1) / threadsPerBlock
	const dim3 blockSize(threadsPerBlock, 1);
	
	//if (hashParams.m_numOccupiedBlocks > 0) {
		garbageCollectFreeKernel << <gridSize, blockSize, 0, stream >> >(hashData);
	//}
#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
#endif
}
