#include "splat_render_cuda.hpp"
//#include <cuda_runtime.h>

#include <ftl/cuda_matrix_util.hpp>

#include "splat_params.hpp"

#define T_PER_BLOCK 8
#define NUM_GROUPS_X 1024

#define NUM_CUDA_BLOCKS  10000

using ftl::cuda::TextureObject;
using ftl::render::SplatParams;

__global__ void clearDepthKernel(ftl::voxhash::HashData hashData, TextureObject<int> depth) {
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < depth.width() && y < depth.height()) {
		depth(x,y) = 0x7f800000; //PINF;
		//colour(x,y) = make_uchar4(76,76,82,0);
	}
}

#define SDF_BLOCK_SIZE_PAD 8
#define SDF_BLOCK_BUFFER 512  // > 8x8x8
#define SDF_DX 1
#define SDF_DY SDF_BLOCK_SIZE_PAD
#define SDF_DZ (SDF_BLOCK_SIZE_PAD*SDF_BLOCK_SIZE_PAD)

#define LOCKED 0x7FFFFFFF

//! computes the (local) virtual voxel pos of an index; idx in [0;511]
__device__ 
int3 pdelinVoxelIndex(uint idx)	{
	int x = idx % SDF_BLOCK_SIZE_PAD;
	int y = (idx % (SDF_BLOCK_SIZE_PAD * SDF_BLOCK_SIZE_PAD)) / SDF_BLOCK_SIZE_PAD;
	int z = idx / (SDF_BLOCK_SIZE_PAD * SDF_BLOCK_SIZE_PAD);	
	return make_int3(x,y,z);
}

//! computes the linearized index of a local virtual voxel pos; pos in [0;7]^3
__device__ 
uint plinVoxelPos(const int3& virtualVoxelPos) {
	return  
		virtualVoxelPos.z * SDF_BLOCK_SIZE_PAD * SDF_BLOCK_SIZE_PAD +
		virtualVoxelPos.y * SDF_BLOCK_SIZE_PAD +
		virtualVoxelPos.x;
}

//! computes the linearized index of a local virtual voxel pos; pos in [0;7]^3
__device__ 
uint plinVoxelPos(int x, int y, int z) {
	return  
		z * SDF_BLOCK_SIZE_PAD * SDF_BLOCK_SIZE_PAD +
		y * SDF_BLOCK_SIZE_PAD + x;
}

__device__  
void deleteVoxel(ftl::voxhash::Voxel& v) {
	v.color = make_uchar3(0,0,0);
	v.weight = 0;
	v.sdf = PINF;
}

__device__ inline int3 blockDelinear(const int3 &base, uint i) {
	return make_int3(base.x + (i & 0x1), base.y + (i & 0x2), base.z + (i & 0x4));
}

__device__ inline uint blockLinear(int x, int y, int z) {
	return x + (y << 1) + (z << 2);
}

__device__ inline bool getVoxel(uint *voxels, int ix) {
	return voxels[ix/32] & (0x1 << (ix % 32));
}

__global__ void occupied_image_kernel(ftl::voxhash::HashData hashData, TextureObject<int> depth, SplatParams params) {
	__shared__ uint voxels[16];
	__shared__ ftl::voxhash::HashEntryHead block;

	// Stride over all allocated blocks
	for (int bi=blockIdx.x; bi<*hashData.d_hashCompactifiedCounter; bi+=NUM_CUDA_BLOCKS) {
	__syncthreads();

	const uint i = threadIdx.x;	//inside of an SDF block

	if (i == 0) block = hashData.d_hashCompactified[bi]->head;
	if (i < 16) {
		voxels[i] = hashData.d_hashCompactified[bi]->voxels[i];
		//valid[i] = hashData.d_hashCompactified[bi]->validity[i];
	}

	// Make sure all hash entries are cached
	__syncthreads();

	const int3 pi_base = hashData.SDFBlockToVirtualVoxelPos(make_int3(block.posXYZ));
	const int3 vp = make_int3(hashData.delinearizeVoxelIndex(i));
	const int3 pi = pi_base + vp;
	const float3 worldPos = hashData.virtualVoxelPosToWorld(pi);

	const bool v = getVoxel(voxels, i);

	uchar4 color = make_uchar4(255,0,0,255);
	bool is_surface = v; //((params.m_flags & ftl::render::kShowBlockBorders) && edgeX + edgeY + edgeZ >= 2);


	// Only for surface voxels, work out screen coordinates
	if (!is_surface) continue;

	// TODO: For each original camera, render a new depth map

	const float3 camPos = params.m_viewMatrix * worldPos;
	const float2 screenPosf = params.camera.cameraToKinectScreenFloat(camPos);
	const uint2 screenPos = make_uint2(make_int2(screenPosf)); //  + make_float2(0.5f, 0.5f)

	//printf("Worldpos: %f,%f,%f\n", camPos.x, camPos.y, camPos.z);

	if (camPos.z < params.camera.m_sensorDepthWorldMin) continue;

	const unsigned int x = screenPos.x;
	const unsigned int y = screenPos.y;
	const int idepth = static_cast<int>(camPos.z * 1000.0f);

	// See: Gunther et al. 2013. A GPGPU-based Pipeline for Accelerated Rendering of Point Clouds
	if (x < depth.width() && y < depth.height()) {
		atomicMin(&depth(x,y), idepth);
	}

	}  // Stride
}

__global__ void isosurface_image_kernel(ftl::voxhash::HashData hashData, TextureObject<int> depth, SplatParams params) {
	// TODO:(Nick) Reduce bank conflicts by aligning these
	__shared__ uint voxels[16];
	//__shared__ uint valid[16];
	__shared__ ftl::voxhash::HashEntryHead block;

	// Stride over all allocated blocks
	for (int bi=blockIdx.x; bi<*hashData.d_hashCompactifiedCounter; bi+=NUM_CUDA_BLOCKS) {
	__syncthreads();

	const uint i = threadIdx.x;	//inside of an SDF block

	if (i == 0) block = hashData.d_hashCompactified[bi]->head;
	if (i < 16) {
		voxels[i] = hashData.d_hashCompactified[bi]->voxels[i];
		//valid[i] = hashData.d_hashCompactified[bi]->validity[i];
	}

	// Make sure all hash entries are cached
	__syncthreads();

	const int3 pi_base = hashData.SDFBlockToVirtualVoxelPos(make_int3(block.posXYZ));
	const int3 vp = make_int3(hashData.delinearizeVoxelIndex(i));
	const int3 pi = pi_base + vp;
	//const uint j = plinVoxelPos(vp);  // Padded linear index
	const float3 worldPos = hashData.virtualVoxelPosToWorld(pi);

	// Load distances and colours into shared memory + padding
	//const ftl::voxhash::Voxel &v = hashData.d_SDFBlocks[block.ptr + i];
	//voxels[j] = v;
	const bool v = getVoxel(voxels, i);

	//__syncthreads();

	//if (voxels[j].weight == 0) continue;
	if (vp.x == 7 || vp.y == 7 || vp.z == 7) continue;


	int edgeX = (vp.x == 0 ) ? 1 : 0;
	int edgeY = (vp.y == 0 ) ? 1 : 0;
	int edgeZ = (vp.z == 0 ) ? 1 : 0;

	uchar4 color = make_uchar4(255,0,0,255);
	bool is_surface = v; //((params.m_flags & ftl::render::kShowBlockBorders) && edgeX + edgeY + edgeZ >= 2);
	//if (is_surface) color = make_uchar4(255,(vp.x == 0 && vp.y == 0 && vp.z == 0) ? 255 : 0,0,255);

	if (v) continue;  // !getVoxel(valid, i)

	//if (vp.z == 7) voxels[j].color = make_uchar3(0,255,(voxels[j].sdf < 0.0f) ? 255 : 0);

	// Identify surfaces through sign change. Since we only check in one direction
	// it is fine to check for any sign change?


#pragma unroll
	for (int u=0; u<=1; u++) {
		for (int v=0; v<=1; v++) {
			for (int w=0; w<=1; w++) {
				const int3 uvi = make_int3(vp.x+u,vp.y+v,vp.z+w);

				// Skip these cases since we didn't load voxels properly
				//if (uvi.x == 8 || uvi.z == 8 || uvi.y == 8) continue;

				const bool vox = getVoxel(voxels, hashData.linearizeVoxelPos(uvi));
				if (vox) { //getVoxel(valid, hashData.linearizeVoxelPos(uvi))) {
					is_surface = true;
					// Should break but is slower?
				}
			}
		}
	}

	// Only for surface voxels, work out screen coordinates
	if (!is_surface) continue;

	// TODO: For each original camera, render a new depth map

	const float3 camPos = params.m_viewMatrix * worldPos;
	const float2 screenPosf = params.camera.cameraToKinectScreenFloat(camPos);
	const uint2 screenPos = make_uint2(make_int2(screenPosf)); //  + make_float2(0.5f, 0.5f)

	//printf("Worldpos: %f,%f,%f\n", camPos.x, camPos.y, camPos.z);

	if (camPos.z < params.camera.m_sensorDepthWorldMin) continue;

	// For this voxel in hash, get its screen position and check it is on screen
	// Convert depth map to int by x1000 and use atomicMin
	//const int pixsize = static_cast<int>((c_hashParams.m_virtualVoxelSize*params.camera.fx/(camPos.z*0.8f)))+1;  // Magic number increase voxel to ensure coverage

	const unsigned int x = screenPos.x;
	const unsigned int y = screenPos.y;
	const int idepth = static_cast<int>(camPos.z * 1000.0f);

	// See: Gunther et al. 2013. A GPGPU-based Pipeline for Accelerated Rendering of Point Clouds
	if (x < depth.width() && y < depth.height()) {
		atomicMin(&depth(x,y), idepth);
	}

	}  // Stride
}

void ftl::cuda::isosurface_point_image(const ftl::voxhash::HashData& hashData,
			const TextureObject<int> &depth,
			const SplatParams &params, cudaStream_t stream) {

	const dim3 clear_gridSize((depth.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 clear_blockSize(T_PER_BLOCK, T_PER_BLOCK);

	clearDepthKernel<<<clear_gridSize, clear_blockSize, 0, stream>>>(hashData, depth);

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
#endif

	const unsigned int threadsPerBlock = SDF_BLOCK_SIZE*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE;
	const dim3 gridSize(NUM_CUDA_BLOCKS, 1);
	const dim3 blockSize(threadsPerBlock, 1);

	occupied_image_kernel<<<gridSize, blockSize, 0, stream>>>(hashData, depth, params);

	cudaSafeCall( cudaGetLastError() );

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
#endif
}


