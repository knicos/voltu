#include <ftl/voxel_hash.hpp>

using namespace ftl::voxhash;

#define COLLISION_LIST_SIZE 5

//! returns the hash entry for a given sdf block id; if there was no hash entry the returned entry will have a ptr with FREE_ENTRY set
__device__ 
HashEntry HashData::getHashEntryForSDFBlockPos(const int3& sdfBlock) const {
	uint h = computeHashPos(sdfBlock); //hash
	int3 pos = sdfBlock;

	HashEntry curr;

	int i = h;
	unsigned int maxIter = 0;

	#pragma unroll 2
	while (maxIter < COLLISION_LIST_SIZE) {
		curr = d_hash[i];

		if (curr.pos == pos && curr.ptr != FREE_ENTRY) return curr;
		if (curr.offset == 0) break;

		i +=  curr.offset;  //go to next element in the list
		i %= (params().m_hashNumBuckets);  //check for overflow
		++maxIter;
	}

	// Could not find so return dummy
	curr.pos = pos;
	curr.ptr = FREE_ENTRY;
	return curr;
}

//for histogram (collisions traversal only)
__device__ 
unsigned int HashData::getNumHashLinkedList(unsigned int bucketID) {
	unsigned int listLen = 0;

	unsigned int i = bucketID;	//start with the last entry of the current bucket
	HashEntry curr;	curr.offset = 0;

	unsigned int maxIter = 0;

	#pragma unroll 2 
	while (maxIter < COLLISION_LIST_SIZE) {
		curr = d_hash[i];

		if (curr.offset == 0) break;

		i += curr.offset;		//go to next element in the list
		i %= (params().m_hashNumBuckets);	//check for overflow
		++listLen;
		++maxIter;
	}
	
	return listLen;
}

//pos in SDF block coordinates
__device__
void HashData::allocBlock(const int3& pos, const uchar frame) {
	uint h = computeHashPos(pos);				//hash bucket
	uint i = h;
	HashEntry curr;	curr.offset = 0;

	unsigned int maxIter = 0;
	#pragma  unroll 2
	while (maxIter < COLLISION_LIST_SIZE) {
		//offset = curr.offset;
		curr = d_hash[i];	//TODO MATTHIAS do by reference
		if (curr.pos == pos && curr.ptr != FREE_ENTRY) return;
		if (curr.offset == 0) break;

		i += curr.offset;		//go to next element in the list
		i %= (params().m_hashNumBuckets);	//check for overflow
		++maxIter;
	}

	// Limit reached...
	if (curr.offset != 0) return;

	int j = i+1;
	while (maxIter < COLLISION_LIST_SIZE) {
		//offset = curr.offset;
		curr = d_hash[j];	//TODO MATTHIAS do by reference
		if (curr.ptr == FREE_ENTRY) {
			int prevValue = atomicExch(&d_hashBucketMutex[i], LOCK_ENTRY);
			if (prevValue != LOCK_ENTRY) {
				//InterlockedExchange(g_HashBucketMutex[h], LOCK_ENTRY, prevValue);	//lock the hash bucket where we have found a free entry
				prevValue = atomicExch(&d_hashBucketMutex[j], LOCK_ENTRY);
				if (prevValue != LOCK_ENTRY) {	//only proceed if the bucket has been locked
					HashEntry& entry = d_hash[j];
					entry.pos = pos;
					entry.offset = 0;
					entry.flags = 0;  // Flag block as valid in this frame (Nick)		
					entry.ptr = consumeHeap() * SDF_BLOCK_SIZE*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE;	//memory alloc
					d_hash[i].offset = j-i;
					//setHashEntry(g_Hash, idxLastEntryInBucket, lastEntryInBucket);
				}
			} 
			return;	//bucket was already locked
		}

		++j;
		j %= (params().m_hashNumBuckets);	//check for overflow
		++maxIter;
	}
}


//!inserts a hash entry without allocating any memory: used by streaming: TODO MATTHIAS check the atomics in this function
/*__device__
bool HashData::insertHashEntry(HashEntry entry)
{
	uint h = computeHashPos(entry.pos);
	uint hp = h * HASH_BUCKET_SIZE;

	for (uint j = 0; j < HASH_BUCKET_SIZE; j++) {
		uint i = j + hp;		
		//const HashEntry& curr = d_hash[i];
		int prevWeight = 0;
		//InterlockedCompareExchange(hash[3*i+2], FREE_ENTRY, LOCK_ENTRY, prevWeight);
		prevWeight = atomicCAS(&d_hash[i].ptr, FREE_ENTRY, LOCK_ENTRY);
		if (prevWeight == FREE_ENTRY) {
			d_hash[i] = entry;
			//setHashEntry(hash, i, entry);
			return true;
		}
	}

#ifdef HANDLE_COLLISIONS
	//updated variables as after the loop
	const uint idxLastEntryInBucket = (h+1)*HASH_BUCKET_SIZE - 1;	//get last index of bucket

	uint i = idxLastEntryInBucket;											//start with the last entry of the current bucket
	HashEntry curr;

	unsigned int maxIter = 0;
	//[allow_uav_condition]
	uint g_MaxLoopIterCount = params().m_hashMaxCollisionLinkedListSize;
	#pragma  unroll 1 
	while (maxIter < g_MaxLoopIterCount) {									//traverse list until end // why find the end? we you are inserting at the start !!!
		//curr = getHashEntry(hash, i);
		curr = d_hash[i];	//TODO MATTHIAS do by reference
		if (curr.offset == 0) break;									//we have found the end of the list
		i = idxLastEntryInBucket + curr.offset;							//go to next element in the list
		i %= (HASH_BUCKET_SIZE * params().m_hashNumBuckets);	//check for overflow

		maxIter++;
	}

	maxIter = 0;
	int offset = 0;
	#pragma  unroll 1 
	while (maxIter < g_MaxLoopIterCount) {													//linear search for free entry
		offset++;
		uint i = (idxLastEntryInBucket + offset) % (HASH_BUCKET_SIZE * params().m_hashNumBuckets);	//go to next hash element
		if ((offset % HASH_BUCKET_SIZE) == 0) continue;										//cannot insert into a last bucket element (would conflict with other linked lists)

		int prevWeight = 0;
		//InterlockedCompareExchange(hash[3*i+2], FREE_ENTRY, LOCK_ENTRY, prevWeight);		//check for a free entry
		uint* d_hashUI = (uint*)d_hash;
		prevWeight = prevWeight = atomicCAS(&d_hashUI[3*idxLastEntryInBucket+1], (uint)FREE_ENTRY, (uint)LOCK_ENTRY);
		if (prevWeight == FREE_ENTRY) {														//if free entry found set prev->next = curr & curr->next = prev->next
			//[allow_uav_condition]
			//while(hash[3*idxLastEntryInBucket+2] == LOCK_ENTRY); // expects setHashEntry to set the ptr last, required because pos.z is packed into the same value -> prev->next = curr -> might corrput pos.z

			HashEntry lastEntryInBucket = d_hash[idxLastEntryInBucket];			//get prev (= lastEntry in Bucket)

			int newOffsetPrev = (offset << 16) | (lastEntryInBucket.pos.z & 0x0000ffff);	//prev->next = curr (maintain old z-pos)
			int oldOffsetPrev = 0;
			//InterlockedExchange(hash[3*idxLastEntryInBucket+1], newOffsetPrev, oldOffsetPrev);	//set prev offset atomically
			uint* d_hashUI = (uint*)d_hash;
			oldOffsetPrev = prevWeight = atomicExch(&d_hashUI[3*idxLastEntryInBucket+1], newOffsetPrev);
			entry.offset = oldOffsetPrev >> 16;													//remove prev z-pos from old offset

			//setHashEntry(hash, i, entry);														//sets the current hashEntry with: curr->next = prev->next
			d_hash[i] = entry;
			return true;
		}

		maxIter++;
	} 
#endif

	return false;
}*/



//! deletes a hash entry position for a given sdfBlock index (returns true uppon successful deletion; otherwise returns false)
__device__
bool HashData::deleteHashEntryElement(const int3& sdfBlock) {
	uint h = computeHashPos(sdfBlock);	//hash bucket

	int i = h;
	int prev = -1;
	HashEntry curr;
	unsigned int maxIter = 0;

	#pragma  unroll 2 
	while (maxIter < COLLISION_LIST_SIZE) {
		curr = d_hash[i];
	
		//found that dude that we need/want to delete
		if (curr.pos == sdfBlock && curr.ptr != FREE_ENTRY) {
			//int prevValue = 0;
			//InterlockedExchange(bucketMutex[h], LOCK_ENTRY, prevValue);	//lock the hash bucket
			int prevValue = atomicExch(&d_hashBucketMutex[i], LOCK_ENTRY);
			if (prevValue == LOCK_ENTRY)	return false;
			if (prevValue != LOCK_ENTRY) {
				prevValue = (prev >= 0) ? atomicExch(&d_hashBucketMutex[prev], LOCK_ENTRY) : 0;
				if (prevValue == LOCK_ENTRY)	return false;
				if (prevValue != LOCK_ENTRY) {
					const uint linBlockSize = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					appendHeap(curr.ptr / linBlockSize);
					deleteHashEntry(i);

					if (prev >= 0) {
						d_hash[prev].offset = curr.offset;
					}
					return true;
				}
			}
		}

		if (curr.offset == 0) {	//we have found the end of the list
			return false;	//should actually never happen because we need to find that guy before
		}
		prev = i;
		i += curr.offset;		//go to next element in the list
		i %= (params().m_hashNumBuckets);	//check for overflow

		++maxIter;
	}

	return false;
}