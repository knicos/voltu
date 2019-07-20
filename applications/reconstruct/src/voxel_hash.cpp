#include <ftl/voxel_hash.hpp>

using ftl::voxhash::HashData;
using ftl::voxhash::HashParams;

void HashData::allocate(const HashParams& params, bool dataOnGPU) {
	m_bIsOnGPU = dataOnGPU;
	if (m_bIsOnGPU) {
		cudaSafeCall(cudaMalloc(&d_hash, sizeof(HashEntry)* params.m_hashNumBuckets));
		cudaSafeCall(cudaMalloc(&d_hashDecision, sizeof(int)* params.m_hashNumBuckets));
		cudaSafeCall(cudaMalloc(&d_hashDecisionPrefix, sizeof(int)* params.m_hashNumBuckets));
		cudaSafeCall(cudaMalloc(&d_hashCompactified, sizeof(HashEntry*)* params.m_hashNumBuckets));
		cudaSafeCall(cudaMalloc(&d_hashCompactifiedCounter, sizeof(int)));
		cudaSafeCall(cudaMalloc(&d_hashBucketMutex, sizeof(int)* params.m_hashNumBuckets));
	} else {
		d_hash = new HashEntry[params.m_hashNumBuckets];
		d_hashDecision = new int[params.m_hashNumBuckets];
		d_hashDecisionPrefix = new int[params.m_hashNumBuckets];
		d_hashCompactified = new HashEntry*[params.m_hashNumBuckets];
		d_hashCompactifiedCounter = new int[1];
		d_hashBucketMutex = new int[params.m_hashNumBuckets];
	}

	updateParams(params);
}

void HashData::updateParams(const HashParams& params) {
	if (m_bIsOnGPU) {
		updateConstantHashParams(params);
	} 
}

void HashData::free() {
	if (m_bIsOnGPU) {
		cudaSafeCall(cudaFree(d_hash));
		cudaSafeCall(cudaFree(d_hashDecision));
		cudaSafeCall(cudaFree(d_hashDecisionPrefix));
		cudaSafeCall(cudaFree(d_hashCompactified));
		cudaSafeCall(cudaFree(d_hashCompactifiedCounter));
		cudaSafeCall(cudaFree(d_hashBucketMutex));
	} else {
		if (d_hash) delete[] d_hash;
		if (d_hashDecision) delete[] d_hashDecision;
		if (d_hashDecisionPrefix) delete[] d_hashDecisionPrefix;
		if (d_hashCompactified) delete[] d_hashCompactified;
		if (d_hashCompactifiedCounter) delete[] d_hashCompactifiedCounter;
		if (d_hashBucketMutex) delete[] d_hashBucketMutex;
	}

	d_hash = NULL;
	d_hashDecision = NULL;
	d_hashDecisionPrefix = NULL;
	d_hashCompactified = NULL;
	d_hashCompactifiedCounter = NULL;
	d_hashBucketMutex = NULL;
}

HashData HashData::download() const {
	if (!m_bIsOnGPU) return *this;
	HashParams params;
	
	HashData hashData;
	hashData.allocate(params, false);	//allocate the data on the CPU
	cudaSafeCall(cudaMemcpy(hashData.d_hash, d_hash, sizeof(HashEntry)* params.m_hashNumBuckets, cudaMemcpyDeviceToHost));
	cudaSafeCall(cudaMemcpy(hashData.d_hashDecision, d_hashDecision, sizeof(int)*params.m_hashNumBuckets, cudaMemcpyDeviceToHost));
	cudaSafeCall(cudaMemcpy(hashData.d_hashDecisionPrefix, d_hashDecisionPrefix, sizeof(int)*params.m_hashNumBuckets, cudaMemcpyDeviceToHost));
	cudaSafeCall(cudaMemcpy(hashData.d_hashCompactified, d_hashCompactified, sizeof(HashEntry*)* params.m_hashNumBuckets, cudaMemcpyDeviceToHost));
	cudaSafeCall(cudaMemcpy(hashData.d_hashCompactifiedCounter, d_hashCompactifiedCounter, sizeof(unsigned int), cudaMemcpyDeviceToHost));
	cudaSafeCall(cudaMemcpy(hashData.d_hashBucketMutex, d_hashBucketMutex, sizeof(int)* params.m_hashNumBuckets, cudaMemcpyDeviceToHost));
	
	return hashData;
}

HashData HashData::upload() const {
	if (m_bIsOnGPU) return *this;
	HashParams params;
	
	HashData hashData;
	hashData.allocate(params, false);	//allocate the data on the CPU
	cudaSafeCall(cudaMemcpy(hashData.d_hash, d_hash, sizeof(HashEntry)* params.m_hashNumBuckets, cudaMemcpyHostToDevice));
	cudaSafeCall(cudaMemcpy(hashData.d_hashDecision, d_hashDecision, sizeof(int)*params.m_hashNumBuckets, cudaMemcpyHostToDevice));
	cudaSafeCall(cudaMemcpy(hashData.d_hashDecisionPrefix, d_hashDecisionPrefix, sizeof(int)*params.m_hashNumBuckets, cudaMemcpyHostToDevice));
	cudaSafeCall(cudaMemcpy(hashData.d_hashCompactified, d_hashCompactified, sizeof(HashEntry)* params.m_hashNumBuckets, cudaMemcpyHostToDevice));
	cudaSafeCall(cudaMemcpy(hashData.d_hashCompactifiedCounter, d_hashCompactifiedCounter, sizeof(unsigned int), cudaMemcpyHostToDevice));
	cudaSafeCall(cudaMemcpy(hashData.d_hashBucketMutex, d_hashBucketMutex, sizeof(int)* params.m_hashNumBuckets, cudaMemcpyHostToDevice));
	
	return hashData;
}

/*size_t HashData::getAllocatedBlocks() const {
	unsigned int count;
	cudaSafeCall(cudaMemcpy(d_heapCounter, &count, sizeof(unsigned int), cudaMemcpyDeviceToHost));
	return count;
}*/
