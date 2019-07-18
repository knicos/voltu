#include "splat_render_cuda.hpp"
#include <cuda_runtime.h>

#include <ftl/cuda_matrix_util.hpp>

#include "splat_params.hpp"

#define T_PER_BLOCK 8
#define NUM_GROUPS_X 1024

#define NUM_CUDA_BLOCKS  10000

using ftl::cuda::TextureObject;
using ftl::render::SplatParams;

__global__ void clearDepthKernel(ftl::voxhash::HashData hashData, TextureObject<uint> depth, TextureObject<uchar4> colour) {
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < depth.width() && y < depth.height()) {
		depth(x,y) = 0x7f800000; //PINF;
		colour(x,y) = make_uchar4(76,76,82,0);
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

__global__ void isosurface_image_kernel(ftl::voxhash::HashData hashData, TextureObject<uint> depth, TextureObject<uchar4> colour, SplatParams params) {
	// TODO:(Nick) Reduce bank conflicts by aligning these
	__shared__ ftl::voxhash::Voxel voxels[SDF_BLOCK_BUFFER];
	__shared__ ftl::voxhash::HashEntry block;

	// Stride over all allocated blocks
	for (int bi=blockIdx.x; bi<*hashData.d_hashCompactifiedCounter; bi+=NUM_CUDA_BLOCKS) {
	__syncthreads();

	const uint i = threadIdx.x;	//inside of an SDF block

	if (i == 0) block = hashData.d_hashCompactified[bi];

	// Make sure all hash entries are cached
	__syncthreads();

	const int3 pi_base = hashData.SDFBlockToVirtualVoxelPos(block.pos);
	const int3 vp = make_int3(hashData.delinearizeVoxelIndex(i));
	const int3 pi = pi_base + vp;
	const uint j = plinVoxelPos(vp);  // Padded linear index
	const float3 worldPos = hashData.virtualVoxelPosToWorld(pi);

	// Load distances and colours into shared memory + padding
	const ftl::voxhash::Voxel &v = hashData.d_SDFBlocks[block.ptr + i];
	voxels[j] = v;

	__syncthreads();

	if (voxels[j].weight == 0) continue;


	int edgeX = (vp.x == 0 ) ? 1 : 0;
	int edgeY = (vp.y == 0 ) ? 1 : 0;
	int edgeZ = (vp.z == 0 ) ? 1 : 0;

	bool is_surface = ((params.m_flags & kShowBlockBorders) && edgeX + edgeY + edgeZ >= 2);
	if (is_surface) voxels[j].color = make_uchar3(255,(vp.x == 0 && vp.y == 0 && vp.z == 0) ? 255 : 0,0);

	if (!is_surface && voxels[j].sdf <= 0.0f) continue;

	//if (vp.z == 7) voxels[j].color = make_uchar3(0,255,(voxels[j].sdf < 0.0f) ? 255 : 0);

	// Identify surfaces through sign change. Since we only check in one direction
	// it is fine to check for any sign change?


#pragma unroll
	for (int u=0; u<=1; u++) {
		for (int v=0; v<=1; v++) {
			for (int w=0; w<=1; w++) {
				const int3 uvi = make_int3(vp.x+u,vp.y+v,vp.z+w);

				// Skip these cases since we didn't load voxels properly
				if (uvi.x == 8 || uvi.z == 8 || uvi.y == 8) continue;

				const auto &vox = voxels[plinVoxelPos(uvi)];
				if (vox.weight > 0 && vox.sdf <= 0.0f) {
					is_surface = true;
					// Should break but is slower?
				}
			}
		}
	}

	// Only for surface voxels, work out screen coordinates
	if (!is_surface) continue;

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
		// TODO:(Nick) Would warp and shared mem considerations improve things?
		// Probably not given we now have thin surfaces...

		// FIXME: Needs a lock here to ensure correct colour is written.
		if (atomicMin(&depth(x,y), idepth) > idepth) {
			colour(x,y) = make_uchar4(voxels[j].color.x, voxels[j].color.y, voxels[j].color.z, 255);
		}

		/*bool p = false;
		while (!p) {
			int ld = atomicExch(&depth(x,y), LOCKED);
			if (ld != LOCKED) {
				p = true;
				if (ld > idepth) {
					colour(x,y) = make_uchar4(voxels[j].color.x, voxels[j].color.y, voxels[j].color.z, 255);
					depth(x,y) = idepth;
				} else {
					depth(x,y) = ld;
				}
			}
		}*/
	}

	}  // Stride
}

void ftl::cuda::isosurface_point_image(const ftl::voxhash::HashData& hashData,
			const TextureObject<uint> &depth,
			const TextureObject<uchar4> &colour,
			const SplatParams &params, cudaStream_t stream) {

	const dim3 clear_gridSize((depth.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 clear_blockSize(T_PER_BLOCK, T_PER_BLOCK);

	clearDepthKernel<<<clear_gridSize, clear_blockSize, 0, stream>>>(hashData, depth, colour);

	//cudaSafeCall( cudaDeviceSynchronize() );

	const unsigned int threadsPerBlock = SDF_BLOCK_SIZE*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE;
	const dim3 gridSize(NUM_CUDA_BLOCKS, 1);
	const dim3 blockSize(threadsPerBlock, 1);

	isosurface_image_kernel<<<gridSize, blockSize, 0, stream>>>(hashData, depth, colour, params);

	cudaSafeCall( cudaGetLastError() );
	//cudaSafeCall( cudaDeviceSynchronize() );
}

// ---- Pass 2: Expand the point splats ----------------------------------------

#define SPLAT_RADIUS 7
#define SPLAT_BOUNDS (2*SPLAT_RADIUS+T_PER_BLOCK+1)
#define SPLAT_BUFFER_SIZE (SPLAT_BOUNDS*SPLAT_BOUNDS)
#define MAX_VALID 8

__device__ float distance2(float3 a, float3 b) {
	const float x = a.x-b.x;
	const float y = a.y-b.y;
	const float z = a.z-b.z;
	return x*x+y*y+z*z;
}

__global__ void splatting_kernel(
		TextureObject<uint> depth_in,
		TextureObject<uchar4> colour_in,
		TextureObject<float> depth_out,
		TextureObject<uchar4> colour_out, SplatParams params) {
	// Read an NxN region and
	// - interpolate a depth value for this pixel
	// - interpolate an rgb value for this pixel
	// Must respect depth discontinuities.
	// How much influence a given neighbour has depends on its depth value

	__shared__ float3 positions[SPLAT_BUFFER_SIZE];

	const int i = threadIdx.y*blockDim.y + threadIdx.x;
	const int bx = blockIdx.x*blockDim.x;
	const int by = blockIdx.y*blockDim.y;
	const int x = bx + threadIdx.x;
	const int y = by + threadIdx.y;

	// const float camMinDepth = params.camera.m_sensorDepthWorldMin;
	// const float camMaxDepth = params.camera.m_sensorDepthWorldMax;

	for (int j=i; j<SPLAT_BUFFER_SIZE; j+=T_PER_BLOCK) {
		const unsigned int sx = (j % SPLAT_BOUNDS)+bx-SPLAT_RADIUS;
		const unsigned int sy = (j / SPLAT_BOUNDS)+by-SPLAT_RADIUS;
		if (sx >= depth_in.width() || sy >= depth_in.height()) {
			positions[j] = make_float3(1000.0f,1000.0f, 1000.0f);
		} else {
			positions[j] = params.camera.kinectDepthToSkeleton(sx, sy, (float)depth_in.tex2D((int)sx,(int)sy) / 1000.0f);
		}
	}

	__syncthreads();

	if (x >= depth_in.width() && y >= depth_in.height()) return;

	const float voxelSquared = params.voxelSize*params.voxelSize;
	float mindepth = 1000.0f;
	int minidx = -1;
	// float3 minpos;

	//float3 validPos[MAX_VALID];
	int validIndices[MAX_VALID];
	int validix = 0;

	for (int v=-SPLAT_RADIUS; v<=SPLAT_RADIUS; ++v) {
		for (int u=-SPLAT_RADIUS; u<=SPLAT_RADIUS; ++u) {
			//const int idx = (threadIdx.y+v)*SPLAT_BOUNDS+threadIdx.x+u;
			const int idx = (threadIdx.y+v+SPLAT_RADIUS)*SPLAT_BOUNDS+threadIdx.x+u+SPLAT_RADIUS;

			float3 posp = positions[idx];
			const float d = posp.z;
			//if (d < camMinDepth || d > camMaxDepth) continue;

			float3 pos = params.camera.kinectDepthToSkeleton(x, y, d);
			float dist = distance2(pos, posp);

			if (dist < voxelSquared) {
				// Valid so check for minimum
				//validPos[validix] = pos;
				validIndices[validix++] = idx;
				if (d < mindepth) {
					mindepth = d;
					minidx = idx;
					// minpos = pos;
				}	
			}
		}
	}

	if (minidx == -1) {
		depth_out(x,y) = 0.0f;
		colour_out(x,y) = make_uchar4(76,76,82,255);
		return;
	}

	float3 colour = make_float3(0.0f, 0.0f, 0.0f);
	float depth = 0.0f;
	float contrib = 0.0f;
	float3 pos = params.camera.kinectDepthToSkeleton(x, y, mindepth);  // TODO:(Nick) Mindepth assumption is poor choice.

	for (int j=0; j<validix; ++j) {
		const int idx = validIndices[j];
		float3 posp = positions[idx];
		//float3 pos = params.camera.kinectDepthToSkeleton(x, y, posp.z);
		float3 delta = (posp - pos) / 2*params.voxelSize;
		float dist = delta.x*delta.x + delta.y*delta.y + delta.z*delta.z;

		// It contributes to pixel
		if (dist < 1.0f && fabs(posp.z - mindepth) < 2*params.voxelSize) {
			const unsigned int sx = (idx % SPLAT_BOUNDS)+bx-SPLAT_RADIUS;
			const unsigned int sy = (idx / SPLAT_BOUNDS)+by-SPLAT_RADIUS;

			// Fast and simple trilinear interpolation
			float c = fabs((1.0f - delta.x) * (1.0f - delta.y) * (1.0f - delta.z));
			uchar4 col = colour_in.tex2D((int)sx, (int)sy);
			colour.x += col.x*c;
			colour.y += col.y*c;
			colour.z += col.z*c;
			contrib += c;
			depth += posp.z * c;
		}
	}

	// Normalise
	colour.x /= contrib;
	colour.y /= contrib;
	colour.z /= contrib;
	depth /= contrib;

	depth_out(x,y) = depth;
	colour_out(x,y) = make_uchar4(colour.x, colour.y, colour.z, 255);
}

void ftl::cuda::splat_points(const TextureObject<uint> &depth_in, const TextureObject<uchar4> &colour_in,
		const TextureObject<float> &depth_out, const TextureObject<uchar4> &colour_out, const SplatParams &params, cudaStream_t stream) 
{

	const dim3 gridSize((depth_in.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth_in.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	splatting_kernel<<<gridSize, blockSize, 0, stream>>>(depth_in, colour_in, depth_out, colour_out, params);
	cudaSafeCall( cudaGetLastError() );
}
