// From: https://github.com/niessner/VoxelHashing/blob/master/DepthSensingCUDA/Source/CUDASceneRepHashSDF.h

#pragma once

#include <cuda_runtime.h>

#include <ftl/configurable.hpp>
#include <ftl/matrix_conversion.hpp>
#include <ftl/voxel_hash.hpp>
#include <ftl/depth_camera.hpp>
#include <unordered_set>
//#include "CUDAScan.h"
// #include "CUDATimer.h"

// #include "GlobalAppState.h"
// #include "TimingLog.h"

#define 	SAFE_DELETE_ARRAY(a)   { delete [] (a); (a) = NULL; }

extern "C" void resetCUDA(ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams);
extern "C" void resetHashBucketMutexCUDA(ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams);
extern "C" void allocCUDA(ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams, const DepthCameraData& depthCameraData, const DepthCameraParams& depthCameraParams, const unsigned int* d_bitMask);
extern "C" void fillDecisionArrayCUDA(ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams, const DepthCameraData& depthCameraData);
extern "C" void compactifyHashCUDA(ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams);
extern "C" unsigned int compactifyHashAllInOneCUDA(ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams);
extern "C" void integrateDepthMapCUDA(ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams, const DepthCameraData& depthCameraData, const DepthCameraParams& depthCameraParams);
extern "C" void bindInputDepthColorTextures(const DepthCameraData& depthCameraData);

extern "C" void starveVoxelsKernelCUDA(ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams);
extern "C" void garbageCollectIdentifyCUDA(ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams);
extern "C" void garbageCollectFreeCUDA(ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams);

namespace ftl {
namespace voxhash {

class SceneRep : public ftl::Configurable {
	public:
	SceneRep(nlohmann::json &config) : Configurable(config) {
		REQUIRED({
			{"hashNumBuckets", "Desired hash entries divide bucket size", "number"},
			{"hashMaxCollisionLinkedListSize", "", "number"},
			{"hashNumSDFBlocks", "", "number"},
			{"SDFVoxelSize", "Size in meters of one voxel", "number"},
			{"SDFMaxIntegrationDistance", "", "number"},
			{"SDFTruncation", "Base error size", "number"},
			{"SDFTruncationScale", "Error size scale with depth", "number"},
			{"SDFIntegrationWeightSample", "", "number"},
			{"SDFIntegrationWeightMax", "", "number"}
		});
		create(parametersFromConfig(config));
	}
	~SceneRep() {
		destroy();
	}

	static HashParams parametersFromConfig(nlohmann::json &config) {
		auto &cfg = ftl::config::resolve(config);
		HashParams params;
		// First camera view is set to identity pose to be at the centre of
		// the virtual coordinate space.
		params.m_rigidTransform.setIdentity();
		params.m_rigidTransformInverse.setIdentity();
		params.m_hashNumBuckets = cfg["hashNumBuckets"];
		params.m_hashBucketSize = HASH_BUCKET_SIZE;
		params.m_hashMaxCollisionLinkedListSize = cfg["hashMaxCollisionLinkedListSize"];
		params.m_SDFBlockSize = SDF_BLOCK_SIZE;
		params.m_numSDFBlocks = cfg["hashNumSDFBlocks"];
		params.m_virtualVoxelSize = cfg["SDFVoxelSize"];
		params.m_maxIntegrationDistance = cfg["SDFMaxIntegrationDistance"];
		params.m_truncation = cfg["SDFTruncation"];
		params.m_truncScale = cfg["SDFTruncationScale"];
		params.m_integrationWeightSample = cfg["SDFIntegrationWeightSample"];
		params.m_integrationWeightMax = cfg["SDFIntegrationWeightMax"];
		// Note (Nick): We are not streaming voxels in/out of GPU
		//params.m_streamingVoxelExtents = MatrixConversion::toCUDA(gas.s_streamingVoxelExtents);
		//params.m_streamingGridDimensions = MatrixConversion::toCUDA(gas.s_streamingGridDimensions);
		//params.m_streamingMinGridPos = MatrixConversion::toCUDA(gas.s_streamingMinGridPos);
		//params.m_streamingInitialChunkListSize = gas.s_streamingInitialChunkListSize;
		return params;
	}

	void bindDepthCameraTextures(const DepthCameraData& depthCameraData) {
		//bindInputDepthColorTextures(depthCameraData);
	}

	/**
	 * Note: lastRigidTransform appears to be the estimated camera pose.
	 * Note: bitMask can be nullptr if not streaming out voxels from GPU
	 */
	void integrate(const Eigen::Matrix4f& lastRigidTransform, const DepthCameraData& depthCameraData, const DepthCameraParams& depthCameraParams, unsigned int* d_bitMask) {
		
		setLastRigidTransform(lastRigidTransform);

		//make the rigid transform available on the GPU
		m_hashData.updateParams(m_hashParams);

		//allocate all hash blocks which are corresponding to depth map entries
		alloc(depthCameraData, depthCameraParams, d_bitMask);

		//generate a linear hash array with only occupied entries
		compactifyHashEntries(depthCameraData);

		//volumetrically integrate the depth data into the depth SDFBlocks
		integrateDepthMap(depthCameraData, depthCameraParams);

		garbageCollect(depthCameraData);

		m_numIntegratedFrames++;
	}

	void setLastRigidTransform(const Eigen::Matrix4f& lastRigidTransform) {
		m_hashParams.m_rigidTransform = MatrixConversion::toCUDA(lastRigidTransform);
		m_hashParams.m_rigidTransformInverse = m_hashParams.m_rigidTransform.getInverse();
	}

	void setLastRigidTransformAndCompactify(const Eigen::Matrix4f& lastRigidTransform, const DepthCameraData& depthCameraData) {
		setLastRigidTransform(lastRigidTransform);
		compactifyHashEntries(depthCameraData);
	}


	const Eigen::Matrix4f getLastRigidTransform() const {
		return MatrixConversion::toEigen(m_hashParams.m_rigidTransform);
	}

	/* Nick: To reduce weights between frames */
	void nextFrame() {
		starveVoxelsKernelCUDA(m_hashData, m_hashParams);
		m_numIntegratedFrames = 0;
	}

	//! resets the hash to the initial state (i.e., clears all data)
	void reset() {
		m_numIntegratedFrames = 0;

		m_hashParams.m_rigidTransform.setIdentity();
		m_hashParams.m_rigidTransformInverse.setIdentity();
		m_hashParams.m_numOccupiedBlocks = 0;
		m_hashData.updateParams(m_hashParams);
		resetCUDA(m_hashData, m_hashParams);
	}


	ftl::voxhash::HashData& getHashData() {
		return m_hashData;
	} 

	const HashParams& getHashParams() const {
		return m_hashParams;
	}


	//! debug only!
	unsigned int getHeapFreeCount() {
		unsigned int count;
		cudaSafeCall(cudaMemcpy(&count, m_hashData.d_heapCounter, sizeof(unsigned int), cudaMemcpyDeviceToHost));
		return count+1;	//there is one more free than the address suggests (0 would be also a valid address)
	}

	//! debug only!
	void debugHash() {
		HashEntry* hashCPU = new HashEntry[m_hashParams.m_hashBucketSize*m_hashParams.m_hashNumBuckets];
		unsigned int* heapCPU = new unsigned int[m_hashParams.m_numSDFBlocks];
		unsigned int heapCounterCPU;

		cudaSafeCall(cudaMemcpy(&heapCounterCPU, m_hashData.d_heapCounter, sizeof(unsigned int), cudaMemcpyDeviceToHost));
		heapCounterCPU++;	//points to the first free entry: number of blocks is one more

		cudaSafeCall(cudaMemcpy(heapCPU, m_hashData.d_heap, sizeof(unsigned int)*m_hashParams.m_numSDFBlocks, cudaMemcpyDeviceToHost));
		cudaSafeCall(cudaMemcpy(hashCPU, m_hashData.d_hash, sizeof(HashEntry)*m_hashParams.m_hashBucketSize*m_hashParams.m_hashNumBuckets, cudaMemcpyDeviceToHost));

		//Check for duplicates
		class myint3Voxel {
		public:
			myint3Voxel() {}
			~myint3Voxel() {}
			bool operator<(const myint3Voxel& other) const {
				if (x == other.x) {
					if (y == other.y) {
						return z < other.z;
					}
					return y < other.y;
				}
				return x < other.x;
			}

			bool operator==(const myint3Voxel& other) const {
				return x == other.x && y == other.y && z == other.z;
			}

			int x,y,z, i;
			int offset;
			int ptr;
		}; 


		std::unordered_set<unsigned int> pointersFreeHash;
		std::vector<int> pointersFreeVec(m_hashParams.m_numSDFBlocks, 0);  // CHECK Nick Changed to int from unsigned in
		for (unsigned int i = 0; i < heapCounterCPU; i++) {
			pointersFreeHash.insert(heapCPU[i]);
			pointersFreeVec[heapCPU[i]] = FREE_ENTRY;
		}
		if (pointersFreeHash.size() != heapCounterCPU) {
			throw std::runtime_error("ERROR: duplicate free pointers in heap array");
		}
		 

		unsigned int numOccupied = 0;
		unsigned int numMinusOne = 0;
		//unsigned int listOverallFound = 0;

		std::list<myint3Voxel> l;
		//std::vector<myint3Voxel> v;
		
		for (unsigned int i = 0; i < m_hashParams.m_hashBucketSize*m_hashParams.m_hashNumBuckets; i++) {
			if (hashCPU[i].ptr == -1) {
				numMinusOne++;
			}

			if (hashCPU[i].ptr != -2) {
				numOccupied++;	// != FREE_ENTRY
				myint3Voxel a;	
				a.x = hashCPU[i].pos.x;
				a.y = hashCPU[i].pos.y;
				a.z = hashCPU[i].pos.z;
				l.push_back(a);
				//v.push_back(a);

				unsigned int linearBlockSize = m_hashParams.m_SDFBlockSize*m_hashParams.m_SDFBlockSize*m_hashParams.m_SDFBlockSize;
				if (pointersFreeHash.find(hashCPU[i].ptr / linearBlockSize) != pointersFreeHash.end()) {
					throw std::runtime_error("ERROR: ptr is on free heap, but also marked as an allocated entry");
				}
				pointersFreeVec[hashCPU[i].ptr / linearBlockSize] = LOCK_ENTRY;
			}
		}

		unsigned int numHeapFree = 0;
		unsigned int numHeapOccupied = 0;
		for (unsigned int i = 0; i < m_hashParams.m_numSDFBlocks; i++) {
			if		(pointersFreeVec[i] == FREE_ENTRY) numHeapFree++;
			else if (pointersFreeVec[i] == LOCK_ENTRY) numHeapOccupied++;
			else {
				throw std::runtime_error("memory leak detected: neither free nor allocated");
			}
		}
		if (numHeapFree + numHeapOccupied == m_hashParams.m_numSDFBlocks) std::cout << "HEAP OK!" << std::endl;
		else throw std::runtime_error("HEAP CORRUPTED");

		l.sort();
		size_t sizeBefore = l.size();
		l.unique();
		size_t sizeAfter = l.size();


		std::cout << "diff: " << sizeBefore - sizeAfter << std::endl;
		std::cout << "minOne: " << numMinusOne << std::endl;
		std::cout << "numOccupied: " << numOccupied << "\t numFree: " << getHeapFreeCount() << std::endl;
		std::cout << "numOccupied + free: " << numOccupied + getHeapFreeCount() << std::endl;
		std::cout << "numInFrustum: " << m_hashParams.m_numOccupiedBlocks << std::endl;

		SAFE_DELETE_ARRAY(heapCPU);
		SAFE_DELETE_ARRAY(hashCPU);

		//getchar();
	}
private:

	void create(const HashParams& params) {
		m_hashParams = params;
		m_hashData.allocate(m_hashParams);

		reset();
	}

	void destroy() {
		m_hashData.free();
	}

	void alloc(const DepthCameraData& depthCameraData, const DepthCameraParams& depthCameraParams, const unsigned int* d_bitMask) {
		//Start Timing
		//if (GlobalAppState::get().s_timingsDetailledEnabled) { cutilSafeCall(cudaDeviceSynchronize()); m_timer.start(); }

		// NOTE (nick): We might want this later...
		if (true) {
			//allocate until all blocks are allocated
			unsigned int prevFree = getHeapFreeCount();
			while (1) {
				resetHashBucketMutexCUDA(m_hashData, m_hashParams);
				allocCUDA(m_hashData, m_hashParams, depthCameraData, depthCameraParams, d_bitMask);

				unsigned int currFree = getHeapFreeCount();

				if (prevFree != currFree) {
					prevFree = currFree;
				}
				else {
					break;
				}
			}
		}
		else {
			//this version is faster, but it doesn't guarantee that all blocks are allocated (staggers alloc to the next frame)
			resetHashBucketMutexCUDA(m_hashData, m_hashParams);
			allocCUDA(m_hashData, m_hashParams, depthCameraData, depthCameraParams, d_bitMask);
		}




		// Stop Timing
		//if(GlobalAppState::get().s_timingsDetailledEnabled) { cutilSafeCall(cudaDeviceSynchronize()); m_timer.stop(); TimingLog::totalTimeAlloc += m_timer.getElapsedTimeMS(); TimingLog::countTimeAlloc++; }
	}


	void compactifyHashEntries(const DepthCameraData& depthCameraData) {
		//Start Timing
		//if(GlobalAppState::get().s_timingsDetailledEnabled) { cutilSafeCall(cudaDeviceSynchronize()); m_timer.start(); }

		//CUDATimer t;

		//t.startEvent("fillDecisionArray");
		//fillDecisionArrayCUDA(m_hashData, m_hashParams, depthCameraData);
		//t.endEvent();

		//t.startEvent("prefixSum");
		//m_hashParams.m_numOccupiedBlocks = 
		//	m_cudaScan.prefixSum(
		//		m_hashParams.m_hashNumBuckets*m_hashParams.m_hashBucketSize,
		//		m_hashData.d_hashDecision,
		//		m_hashData.d_hashDecisionPrefix);
		//t.endEvent();

		//t.startEvent("compactifyHash");
		//m_hashData.updateParams(m_hashParams);	//make sure numOccupiedBlocks is updated on the GPU
		//compactifyHashCUDA(m_hashData, m_hashParams);
		//t.endEvent();

		//t.startEvent("compactifyAllInOne");
		m_hashParams.m_numOccupiedBlocks = compactifyHashAllInOneCUDA(m_hashData, m_hashParams);		//this version uses atomics over prefix sums, which has a much better performance
		std::cout << "Occ blocks = " << m_hashParams.m_numOccupiedBlocks << std::endl;
		m_hashData.updateParams(m_hashParams);	//make sure numOccupiedBlocks is updated on the GPU
		//t.endEvent();
		//t.evaluate();

		// Stop Timing
		//if(GlobalAppState::get().s_timingsDetailledEnabled) { cutilSafeCall(cudaDeviceSynchronize()); m_timer.stop(); TimingLog::totalTimeCompactifyHash += m_timer.getElapsedTimeMS(); TimingLog::countTimeCompactifyHash++; }

		//std::cout << "numOccupiedBlocks: " << m_hashParams.m_numOccupiedBlocks << std::endl;
	}

	void integrateDepthMap(const DepthCameraData& depthCameraData, const DepthCameraParams& depthCameraParams) {
		//Start Timing
		//if(GlobalAppState::get().s_timingsDetailledEnabled) { cutilSafeCall(cudaDeviceSynchronize()); m_timer.start(); }

		integrateDepthMapCUDA(m_hashData, m_hashParams, depthCameraData, depthCameraParams);

		// Stop Timing
		//if(GlobalAppState::get().s_timingsDetailledEnabled) { cutilSafeCall(cudaDeviceSynchronize()); m_timer.stop(); TimingLog::totalTimeIntegrate += m_timer.getElapsedTimeMS(); TimingLog::countTimeIntegrate++; }
	}

	void garbageCollect(const DepthCameraData& depthCameraData) {
		//only perform if enabled by global app state
		//if (GlobalAppState::get().s_garbageCollectionEnabled) {

			garbageCollectIdentifyCUDA(m_hashData, m_hashParams);
			resetHashBucketMutexCUDA(m_hashData, m_hashParams);	//needed if linked lists are enabled -> for memeory deletion
			garbageCollectFreeCUDA(m_hashData, m_hashParams);
		//} 
	}



	HashParams		m_hashParams;
	ftl::voxhash::HashData		m_hashData;

	//CUDAScan		m_cudaScan;
	unsigned int	m_numIntegratedFrames;	//used for garbage collect

	// static Timer m_timer;
};

};  // namespace voxhash
};  // namespace ftl
