#include <ftl/voxel_hash.hpp>
#include "garbage.hpp"

using namespace ftl::voxhash;

#define T_PER_BLOCK 8
#define NUM_CUDA_BLOCKS	10000

/*__global__ void starveVoxelsKernel(HashData hashData) {
	int ptr;

	// Stride over all allocated blocks
	for (int bi=blockIdx.x; bi<*hashData.d_hashCompactifiedCounter; bi+=NUM_CUDA_BLOCKS) {

	ptr = hashData.d_hashCompactified[bi].ptr;
	int weight = hashData.d_SDFBlocks[ptr + threadIdx.x].weight;
	weight = max(0, weight-2);	
	hashData.d_SDFBlocks[ptr + threadIdx.x].weight = weight;  //CHECK Remove to totally clear previous frame (Nick)

	}
}

void ftl::cuda::starveVoxels(HashData& hashData, const HashParams& hashParams, cudaStream_t stream) {
	const unsigned int threadsPerBlock = SDF_BLOCK_SIZE*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE;
	const dim3 gridSize(NUM_CUDA_BLOCKS, 1);
	const dim3 blockSize(threadsPerBlock, 1);

	//if (hashParams.m_numOccupiedBlocks > 0) {
		starveVoxelsKernel << <gridSize, blockSize, 0, stream >> >(hashData);
	//}
#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
#endif
}*/

#define ENTRIES_PER_BLOCK 4

__global__ void clearVoxelsKernel(HashData hashData) {
	const int lane = threadIdx.x % 16;
	const int halfWarp = threadIdx.x / 16;

	// Stride over all allocated blocks
	for (int bi=blockIdx.x+halfWarp; bi<*hashData.d_hashCompactifiedCounter; bi+=NUM_CUDA_BLOCKS*ENTRIES_PER_BLOCK) {

	HashEntry *entry = hashData.d_hashCompactified[bi];	
	//hashData.d_SDFBlocks[entry.ptr + threadIdx.x].weight = 0;
	entry->voxels[lane] = 0;

	}
}

void ftl::cuda::clearVoxels(HashData& hashData, const HashParams& hashParams) {
	const unsigned int threadsPerBlock = 16 * ENTRIES_PER_BLOCK;
	const dim3 gridSize(NUM_CUDA_BLOCKS, 1);
	const dim3 blockSize(threadsPerBlock, 1);

	clearVoxelsKernel << <gridSize, blockSize >> >(hashData);
}


__global__ void garbageCollectIdentifyKernel(HashData hashData) {
	const int lane = threadIdx.x % 16;
	const int halfWarp = threadIdx.x / 16;

	// Stride over all allocated blocks
	for (int bi=blockIdx.x+halfWarp; bi<*hashData.d_hashCompactifiedCounter; bi+=NUM_CUDA_BLOCKS * ENTRIES_PER_BLOCK) {

	const HashEntry *entry = hashData.d_hashCompactified[bi];

	const uint v = entry->voxels[lane];
	const uint mask = (halfWarp & 0x1) ? 0xFFFF0000 : 0x0000FFFF;
	uint ballot_result = __ballot_sync(mask, v == 0 || v == 0xFFFFFFFF);

	if (lane == 0) hashData.d_hashDecision[bi] = (ballot_result == mask) ? 1 : 0;

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
	for (int bi=blockIdx.x*blockDim.x + threadIdx.x; bi<*hashData.d_hashCompactifiedCounter; bi+=NUM_CUDA_BLOCKS*blockDim.x) {

	HashEntry *entry = hashData.d_hashCompactified[bi];

	if ((entry->head.flags & ftl::voxhash::kFlagSurface) == 0) {	//decision to delete the hash entry

		
		//if (entry->head.offset == FREE_ENTRY) return; //should never happen since we did compactify before

		int3 posI3 = make_int3(entry->head.posXYZ.x, entry->head.posXYZ.y, entry->head.posXYZ.z);

		if (hashData.deleteHashEntryElement(posI3)) {	//delete hash entry from hash (and performs heap append)
			//#pragma unroll
			//for (uint i = 0; i < 16; i++) {	//clear sdf block: CHECK TODO another kernel?
			//	entry->voxels[i] = 0;
			//}
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
