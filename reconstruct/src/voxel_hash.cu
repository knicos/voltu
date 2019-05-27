#include <ftl/voxel_hash.hpp>

using ftl::voxhash::VoxelHash;
using ftl::voxhash::hash;
using ftl::voxhash::HashBucket;
using ftl::voxhash::HashEntry;

__device__ HashEntry *ftl::voxhash::cuda::lookupBlock(HashBucket *table, int3 vox) {
	uint32_t hashcode = hash<HASH_SIZE>(vox.x / 8, vox.y / 8, vox.z / 8);
	HashBucket *bucket = table[hashcode];

	while (bucket) {
		for (int i=0; i<ENTRIES_PER_BUCKET; i++) {
			HashEntry *entry = &bucket->entries[i];
			if (entry->position[0] == vox.x && entry->position[1] == vox.y && entry->position[2] == vox.z) {
				return entry;
			}
		}
		if (bucket->entries[ENTRIES_PER_BUCKET-1].offset) {
			bucket += bucket->entries[ENTRIES_PER_BUCKET-1].offset;
		} else {
			break;
		}
	}

	return nullptr;
}

__device__ HashEntry *ftl::voxhash::cuda::insertBlock(HashBucket *table, int3 vox) {
	uint32_t hashcode = hash<HASH_SIZE>(vox.x / 8, vox.y / 8, vox.z / 8);
	HashBucket *bucket = table[hashcode];

	while (bucket) {
		for (int i=0; i<ENTRIES_PER_BUCKET; i++) {
			HashEntry *entry = &bucket->entries[i];
			if (entry->position[0] == vox.x && entry->position[1] == vox.y && entry->position[2] == vox.z) {
				return entry;
			} else if (entry->pointer == 0) {
				// Allocate block.
				entry->position[0] = vox.x;
				entry->position[1] = vox.y;
				entry->position[2] = vox.z;
			}
		}
		if (bucket->entries[ENTRIES_PER_BUCKET-1].offset) {
			bucket += bucket->entries[ENTRIES_PER_BUCKET-1].offset;
		} else {
			// Find a new bucket to append to this list
		}
	}

	return nullptr;
}

__device__ Voxel *ftl::voxhash::cuda::insert(HashBucket *table, VoxelBlocks *blocks, int3 vox) {
	HashEntry *entry = insertBlock(table, vox);
	VoxelBlock *block = blocks[entry->pointer];
	return &block->voxels[vox.x % 8][vox.y % 8][vox.z % 8];
}

__device__ Voxel *ftl::voxhash::cuda::lookup(HashBucket *table, int3 vox) {
	HashEntry *entry = lookupBlock(table, vox);
	if (entry == nullptr) return nullptr;
	VoxelBlock *block = blocks[entry->pointer];
	return &block->voxels[vox.x % 8][vox.y % 8][vox.z % 8];
}

VoxelHash::VoxelHash(float resolution) : resolution_(resolution) {
	// Allocate CPU memory
	table_cpu_.resize(HASH_SIZE);

	// Allocate and init GPU memory
	table_gpu_ = table_cpu_;
}

VoxelHash::~VoxelHash() {

}

void VoxelHash::nextFrame() {
	// Clear the hash for now...
}

__global__ static void merge_view_kernel(PtrStepSz<cv::Point3f> cloud, PtrStepSz<uchar3> rgb, PtrStepSz<double> pose, ftl::voxhash::cuda::VoxelHash vhash) {
	for (STRIDE_Y(v,cloud.rows) {
		for (STRIDE_X(u,cloud.cols) {
			cv::Point3f p = cloud.at<cv::Point3f>(v,u);
			// TODO Lock hash block on insert.
			Voxel *voxel = ftl::voxhash::cuda::insert(vhash.table, make_int3(p.x,p.y,p.z));
			uchar3 colour = rgb.at<uchar3>(v,u);
			voxel->colorRGB[0] = colour[0];
			voxel->colorRGB[1] = colour[1];
			voxel->colorRGB[2] = colour[2];
			voxel->weight = 1;
		}
	}
}

static void merge_view_call(const PtrStepSz<cv::Point3f> &cloud, const PtrStepSz<uchar3> &rgb, const PtrStepSz<double> &pose, ftl::voxhash::cuda::VoxelHash vhash) {
	dim3 grid(1,1,1);
	dim3 threads(128, 1, 1);
	grid.x = cv::cuda::device::divUp(l.cols, 128);
	grid.y = cv::cuda::device::divUp(l.rows, 21);

	merge_view_kernel<<<grid,threads>>>(cloud, rgb, pose, vhash);
	cudaSafeCall( cudaGetLastError() );
}

void VoxelHash::mergeView(const cv::Mat &cloud, const cv::Mat &rgb, const cv::Mat &pose) {
	// Upload GPU Mat
	gpu_cloud_.upload(cloud);
	gpu_rgb_.upload(rgb);
	gpu_pose_.upload(pose);

	// Call kernel to insert each point in hash table
	merge_view_call(gpu_cloud_, gpu_rgb_, gpu_pose_, getCUDAStructure());
}

// Called per voxel
__global__ static void render_kernel(PtrStepSz<uchar3> out, PtrStepSz<double> cam, ftl::voxhash::cuda::VoxelHash vhash) {

}

void VoxelHash::render(cv::Mat &output, const cv::Mat &cam) {
	// Call kernel to process every voxel and add to image
	// Download gpu mat into output.
}
