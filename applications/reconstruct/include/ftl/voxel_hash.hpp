// From: https://github.com/niessner/VoxelHashing/blob/master/DepthSensingCUDA/Source/VoxelUtilHashSDF.h

#pragma once

#ifndef sint
typedef signed int sint;
#endif

#ifndef uint
typedef unsigned int uint;
#endif 

#ifndef slong 
typedef signed long slong;
#endif

#ifndef ulong
typedef unsigned long ulong;
#endif

#ifndef uchar
typedef unsigned char uchar;
#endif

#ifndef schar
typedef signed char schar;
#endif




#include <ftl/cuda_util.hpp>

#include <ftl/cuda_matrix_util.hpp>
#include <ftl/voxel_hash_params.hpp>

#include <ftl/depth_camera.hpp>

#define SDF_BLOCK_SIZE 8

#ifndef MINF
#define MINF __int_as_float(0xff800000)
#endif

#ifndef PINF
#define PINF __int_as_float(0x7f800000)
#endif

extern  __constant__ ftl::voxhash::HashParams c_hashParams;
extern "C" void updateConstantHashParams(const ftl::voxhash::HashParams& hashParams);

namespace ftl {
namespace voxhash {

//status flags for hash entries
static const int LOCK_ENTRY = -1;
static const int FREE_ENTRY = -2;
static const int NO_OFFSET = 0;

struct __align__(16) HashEntry 
{
	int3	pos;		//hash position (lower left corner of SDFBlock))
	int		ptr;		//pointer into heap to SDFBlock
	uint	offset;		//offset for collisions
	uint	flags;		//for interframe checks (Nick)
	
	__device__ void operator=(const struct HashEntry& e) {
		((long long*)this)[0] = ((const long long*)&e)[0];
		((long long*)this)[1] = ((const long long*)&e)[1];
		//((int*)this)[4] = ((const int*)&e)[4];
		((long long*)this)[2] = ((const long long*)&e)[2];
	}
};

struct __align__(8) Voxel {
	float	sdf;		//signed distance function
	uchar3	color;		//color 
	uchar	weight;		//accumulated sdf weight

	__device__ void operator=(const struct Voxel& v) {
		((long long*)this)[0] = ((const long long*)&v)[0];
	}

};
 
/**
 * Voxel Hash Table structure and operations. Works on both CPU and GPU with
 * host <-> device transfer included.
 */
struct HashData {

	///////////////
	// Host part //
	///////////////

	__device__ __host__
	HashData() {
		d_heap = NULL;
		d_heapCounter = NULL;
		d_hash = NULL;
		d_hashDecision = NULL;
		d_hashDecisionPrefix = NULL;
		d_hashCompactified = NULL;
		d_hashCompactifiedCounter = NULL;
		d_SDFBlocks = NULL;
		d_hashBucketMutex = NULL;
		m_bIsOnGPU = false;
	}

	/**
	 * Create all the data structures, either on GPU or system memory.
	 */
	__host__ void allocate(const HashParams& params, bool dataOnGPU = true);

	__host__ void updateParams(const HashParams& params);

	__host__ void free();

	/**
	 * Download entire hash table from GPU to CPU memory.
	 */
	__host__ HashData download() const;

	/**
	 * Upload entire hash table from CPU to GPU memory.
	 */
	__host__ HashData upload() const;

	__host__ size_t getAllocatedBlocks() const;

	__host__ size_t getFreeBlocks() const;

	__host__ size_t getCollisionCount() const;



	/////////////////
	// Device part //
	/////////////////
//#define __CUDACC__
#ifdef __CUDACC__

	__device__
	const HashParams& params() const {
		return c_hashParams;
	}

	//! see teschner et al. (but with correct prime values)
	__device__ 
	uint computeHashPos(const int3& virtualVoxelPos) const { 
		const int p0 = 73856093;
		const int p1 = 19349669;
		const int p2 = 83492791;

		int res = ((virtualVoxelPos.x * p0) ^ (virtualVoxelPos.y * p1) ^ (virtualVoxelPos.z * p2)) % params().m_hashNumBuckets;
		if (res < 0) res += params().m_hashNumBuckets;
		return (uint)res;
	}

	//merges two voxels (v0 the currently stored voxel, v1 is the input voxel)
	__device__ 
	void combineVoxel(const Voxel &v0, const Voxel& v1, Voxel &out) const 	{

		//v.color = (10*v0.weight * v0.color + v1.weight * v1.color)/(10*v0.weight + v1.weight);	//give the currently observed color more weight
		//v.color = (v0.weight * v0.color + v1.weight * v1.color)/(v0.weight + v1.weight);
		//out.color = 0.5f * (v0.color + v1.color);	//exponential running average 
		

		//float3 c0 = make_float3(v0.color.x, v0.color.y, v0.color.z);
		//float3 c1 = make_float3(v1.color.x, v1.color.y, v1.color.z);

		//float3 res = (c0.x+c0.y+c0.z == 0) ? c1 : 0.5f*c0 + 0.5f*c1;
		//float3 res = (c0+c1)/2;
		//float3 res = (c0 * (float)v0.weight + c1 * (float)v1.weight) / ((float)v0.weight + (float)v1.weight);
		//float3 res = c1;

		//out.color.x = (uchar)(res.x+0.5f);	out.color.y = (uchar)(res.y+0.5f); out.color.z = (uchar)(res.z+0.5f);
		
		// Nick: reduces colour flicker but not ideal..
		out.color = v1.color;

		// Option 3 (Nick): Use colour with minimum SDF since it should be closest to surface.
		// Results in stable but pixelated output
		//out.color = (v0.weight > 0 && (fabs(v0.sdf) < fabs(v1.sdf))) ? v0.color : v1.color;

		// Option 4 (Nick): Merge colours based upon relative closeness
		/*float3 c0 = make_float3(v0.color.x, v0.color.y, v0.color.z);
		float3 c1 = make_float3(v1.color.x, v1.color.y, v1.color.z);
		float factor = fabs(v0.sdf - v1.sdf) / 0.05f / 2.0f;
		if (factor > 0.5f) factor = 0.5f;
		float factor0 = (fabs(v0.sdf) < fabs(v1.sdf)) ? 1.0f - factor : factor;
		float factor1 = 1.0f - factor0;
		out.color.x = (v0.weight > 0) ? (uchar)(c0.x * factor0 + c1.x * factor1) : c1.x;
		out.color.y = (v0.weight > 0) ? (uchar)(c0.y * factor0 + c1.y * factor1) : c1.y;
		out.color.z = (v0.weight > 0) ? (uchar)(c0.z * factor0 + c1.z * factor1) : c1.z;*/

		out.sdf = (v0.sdf * (float)v0.weight + v1.sdf * (float)v1.weight) / ((float)v0.weight + (float)v1.weight);
		out.weight = min(params().m_integrationWeightMax, (unsigned int)v0.weight + (unsigned int)v1.weight);
	}


	//! returns the truncation of the SDF for a given distance value
	__device__ 
	float getTruncation(float z) const {
		return params().m_truncation + params().m_truncScale * z;
	}


	__device__ 
	float3 worldToVirtualVoxelPosFloat(const float3& pos) const	{
		return pos / params().m_virtualVoxelSize;
	}

	__device__ 
	int3 worldToVirtualVoxelPos(const float3& pos) const {
		//const float3 p = pos*g_VirtualVoxelResolutionScalar;
		const float3 p = pos / params().m_virtualVoxelSize;
		return make_int3(p+make_float3(sign(p))*0.5f);
	}

	__device__ 
	int3 virtualVoxelPosToSDFBlock(int3 virtualVoxelPos) const {
		if (virtualVoxelPos.x < 0) virtualVoxelPos.x -= SDF_BLOCK_SIZE-1;
		if (virtualVoxelPos.y < 0) virtualVoxelPos.y -= SDF_BLOCK_SIZE-1;
		if (virtualVoxelPos.z < 0) virtualVoxelPos.z -= SDF_BLOCK_SIZE-1;

		return make_int3(
			virtualVoxelPos.x/SDF_BLOCK_SIZE,
			virtualVoxelPos.y/SDF_BLOCK_SIZE,
			virtualVoxelPos.z/SDF_BLOCK_SIZE);
	}

	// Computes virtual voxel position of corner sample position
	__device__ 
	int3 SDFBlockToVirtualVoxelPos(const int3& sdfBlock) const	{
		return sdfBlock*SDF_BLOCK_SIZE;
	}

	__device__ 
	float3 virtualVoxelPosToWorld(const int3& pos) const	{
		return make_float3(pos)*params().m_virtualVoxelSize;
	}

	__device__ 
	float3 SDFBlockToWorld(const int3& sdfBlock) const	{
		return virtualVoxelPosToWorld(SDFBlockToVirtualVoxelPos(sdfBlock));
	}

	__device__ 
	int3 worldToSDFBlock(const float3& worldPos) const	{
		return virtualVoxelPosToSDFBlock(worldToVirtualVoxelPos(worldPos));
	}

	__device__
	bool isSDFBlockInCameraFrustumApprox(const HashParams &hashParams, const DepthCameraParams &camera, const int3& sdfBlock) {
		// NOTE (Nick): Changed, just assume all voxels are potentially in frustrum
		//float3 posWorld = virtualVoxelPosToWorld(SDFBlockToVirtualVoxelPos(sdfBlock)) + hashParams.m_virtualVoxelSize * 0.5f * (SDF_BLOCK_SIZE - 1.0f);
		//return camera.isInCameraFrustumApprox(hashParams.m_rigidTransformInverse, posWorld);
		return true;
	}

	//! computes the (local) virtual voxel pos of an index; idx in [0;511]
	__device__ 
	uint3 delinearizeVoxelIndex(uint idx) const	{
		uint x = idx % SDF_BLOCK_SIZE;
		uint y = (idx % (SDF_BLOCK_SIZE * SDF_BLOCK_SIZE)) / SDF_BLOCK_SIZE;
		uint z = idx / (SDF_BLOCK_SIZE * SDF_BLOCK_SIZE);	
		return make_uint3(x,y,z);
	}

	//! computes the linearized index of a local virtual voxel pos; pos in [0;7]^3
	__device__ 
	uint linearizeVoxelPos(const int3& virtualVoxelPos)	const {
		return  
			virtualVoxelPos.z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE +
			virtualVoxelPos.y * SDF_BLOCK_SIZE +
			virtualVoxelPos.x;
	}

	__device__ 
	int virtualVoxelPosToLocalSDFBlockIndex(const int3& virtualVoxelPos) const	{
		int3 localVoxelPos = make_int3(
			virtualVoxelPos.x % SDF_BLOCK_SIZE,
			virtualVoxelPos.y % SDF_BLOCK_SIZE,
			virtualVoxelPos.z % SDF_BLOCK_SIZE);

		if (localVoxelPos.x < 0) localVoxelPos.x += SDF_BLOCK_SIZE;
		if (localVoxelPos.y < 0) localVoxelPos.y += SDF_BLOCK_SIZE;
		if (localVoxelPos.z < 0) localVoxelPos.z += SDF_BLOCK_SIZE;

		return linearizeVoxelPos(localVoxelPos);
	}

	__device__ 
	int worldToLocalSDFBlockIndex(const float3& world) const	{
		int3 virtualVoxelPos = worldToVirtualVoxelPos(world);
		return virtualVoxelPosToLocalSDFBlockIndex(virtualVoxelPos);
	}


		//! returns the hash entry for a given worldPos; if there was no hash entry the returned entry will have a ptr with FREE_ENTRY set
	__device__ 
	HashEntry getHashEntry(const float3& worldPos) const	{
		//int3 blockID = worldToSDFVirtualVoxelPos(worldPos)/SDF_BLOCK_SIZE;	//position of sdf block
		int3 blockID = worldToSDFBlock(worldPos);
		return getHashEntryForSDFBlockPos(blockID);
	}


	__device__ 
		void deleteHashEntry(uint id) {
			deleteHashEntry(d_hash[id]);
	}

	__device__ 
		void deleteHashEntry(HashEntry& hashEntry) {
			hashEntry.pos = make_int3(0);
			hashEntry.offset = 0;
			hashEntry.ptr = FREE_ENTRY;
	}

	__device__ 
		bool voxelExists(const float3& worldPos) const	{
			HashEntry hashEntry = getHashEntry(worldPos);
			return (hashEntry.ptr != FREE_ENTRY);
	}

	__device__  
	void deleteVoxel(Voxel& v) const {
		v.color = make_uchar3(0,0,0);
		v.weight = 0;
		v.sdf = 0.0f;
	}
	__device__ 
		void deleteVoxel(uint id) {
			deleteVoxel(d_SDFBlocks[id]);
	}


	__device__ 
	Voxel getVoxel(const float3& worldPos) const	{
		HashEntry hashEntry = getHashEntry(worldPos);
		Voxel v;
		if (hashEntry.ptr == FREE_ENTRY) {
			deleteVoxel(v);			
		} else {
			int3 virtualVoxelPos = worldToVirtualVoxelPos(worldPos);
			v = d_SDFBlocks[hashEntry.ptr + virtualVoxelPosToLocalSDFBlockIndex(virtualVoxelPos)];
		}
		return v;
	}

	__device__ 
	Voxel getVoxel(const int3& virtualVoxelPos) const	{
		HashEntry hashEntry = getHashEntryForSDFBlockPos(virtualVoxelPosToSDFBlock(virtualVoxelPos));
		Voxel v;
		if (hashEntry.ptr == FREE_ENTRY) {
			deleteVoxel(v);			
		} else {
			v = d_SDFBlocks[hashEntry.ptr + virtualVoxelPosToLocalSDFBlockIndex(virtualVoxelPos)];
		}
		return v;
	}
	
	__device__ 
	void setVoxel(const int3& virtualVoxelPos, Voxel& voxelInput) const {
		HashEntry hashEntry = getHashEntryForSDFBlockPos(virtualVoxelPosToSDFBlock(virtualVoxelPos));
		if (hashEntry.ptr != FREE_ENTRY) {
			d_SDFBlocks[hashEntry.ptr + virtualVoxelPosToLocalSDFBlockIndex(virtualVoxelPos)] = voxelInput;
		}
	}

	//! returns the hash entry for a given sdf block id; if there was no hash entry the returned entry will have a ptr with FREE_ENTRY set
	__device__ 
	HashEntry getHashEntryForSDFBlockPos(const int3& sdfBlock) const;

	//for histogram (no collision traversal)
	__device__ 
	unsigned int getNumHashEntriesPerBucket(unsigned int bucketID);

	//for histogram (collisions traversal only)
	__device__ 
	unsigned int getNumHashLinkedList(unsigned int bucketID);


	__device__
	uint consumeHeap() {
		uint addr = atomicSub(&d_heapCounter[0], 1);
		//TODO MATTHIAS check some error handling?
		return d_heap[addr];
	}
	__device__
	void appendHeap(uint ptr) {
		uint addr = atomicAdd(&d_heapCounter[0], 1);
		//TODO MATTHIAS check some error handling?
		d_heap[addr+1] = ptr;
	}

	//pos in SDF block coordinates
	__device__
	void allocBlock(const int3& pos, const uchar frame);

	//!inserts a hash entry without allocating any memory: used by streaming: TODO MATTHIAS check the atomics in this function
	__device__
	bool insertHashEntry(HashEntry entry);

	//! deletes a hash entry position for a given sdfBlock index (returns true uppon successful deletion; otherwise returns false)
	__device__
	bool deleteHashEntryElement(const int3& sdfBlock);

#endif	//CUDACC

	uint*		d_heap;						//heap that manages free memory
	uint*		d_heapCounter;				//single element; used as an atomic counter (points to the next free block)
	int*		d_hashDecision;				//
	int*		d_hashDecisionPrefix;		//
	HashEntry*	d_hash;						//hash that stores pointers to sdf blocks
	HashEntry*	d_hashCompactified;			//same as before except that only valid pointers are there
	int*		d_hashCompactifiedCounter;	//atomic counter to add compactified entries atomically 
	Voxel*		d_SDFBlocks;				//sub-blocks that contain 8x8x8 voxels (linearized); are allocated by heap
	int*		d_hashBucketMutex;			//binary flag per hash bucket; used for allocation to atomically lock a bucket

	bool		m_bIsOnGPU;					//the class be be used on both cpu and gpu
};

}  // namespace voxhash
}  // namespace ftl
