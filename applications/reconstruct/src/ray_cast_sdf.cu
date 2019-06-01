
#include <cuda_runtime.h>

#include <ftl/cuda_matrix_util.hpp>

#include <ftl/depth_camera.hpp>
#include <ftl/voxel_hash.hpp>
#include <ftl/ray_cast_util.hpp>

#define T_PER_BLOCK 8
#define NUM_GROUPS_X 1024

//texture<float, cudaTextureType2D, cudaReadModeElementType> rayMinTextureRef;
//texture<float, cudaTextureType2D, cudaReadModeElementType> rayMaxTextureRef;

__global__ void renderKernel(ftl::voxhash::HashData hashData, RayCastData rayCastData) 
{
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	const RayCastParams& rayCastParams = c_rayCastParams;

	if (x < rayCastParams.m_width && y < rayCastParams.m_height) {
		rayCastData.d_depth[y*rayCastParams.m_width+x] = MINF;
		rayCastData.d_depth3[y*rayCastParams.m_width+x] = make_float3(MINF,MINF,MINF);
		rayCastData.d_normals[y*rayCastParams.m_width+x] = make_float4(MINF,MINF,MINF,MINF);
		rayCastData.d_colors[y*rayCastParams.m_width+x] = make_uchar3(0,0,0);

		float3 camDir = normalize(DepthCameraData::kinectProjToCamera(x, y, 1.0f));
		float3 worldCamPos = rayCastParams.m_viewMatrixInverse * make_float3(0.0f, 0.0f, 0.0f);
		float4 w = rayCastParams.m_viewMatrixInverse * make_float4(camDir, 0.0f);
		float3 worldDir = normalize(make_float3(w.x, w.y, w.z));

		////use ray interval splatting
		//float minInterval = tex2D(rayMinTextureRef, x, y);
		//float maxInterval = tex2D(rayMaxTextureRef, x, y);

		//don't use ray interval splatting
		float minInterval = rayCastParams.m_minDepth;
		float maxInterval = rayCastParams.m_maxDepth;

		//if (minInterval == 0 || minInterval == MINF) minInterval = rayCastParams.m_minDepth;
		//if (maxInterval == 0 || maxInterval == MINF) maxInterval = rayCastParams.m_maxDepth;
		//TODO MATTHIAS: shouldn't this return in the case no interval is found?
		if (minInterval == 0 || minInterval == MINF) return;
		if (maxInterval == 0 || maxInterval == MINF) return;

		// debugging 
		//if (maxInterval < minInterval) {
		//	printf("ERROR (%d,%d): [ %f, %f ]\n", x, y, minInterval, maxInterval);
		//}

		rayCastData.traverseCoarseGridSimpleSampleAll(hashData, worldCamPos, worldDir, camDir, make_int3(x,y,1), minInterval, maxInterval);
	} 
}

extern "C" void renderCS(const ftl::voxhash::HashData& hashData, const RayCastData &rayCastData, const RayCastParams &rayCastParams) 
{

	const dim3 gridSize((rayCastParams.m_width + T_PER_BLOCK - 1)/T_PER_BLOCK, (rayCastParams.m_height + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	//cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc(32, 0, 0, 0, cudaChannelFormatKindFloat);
	//cudaBindTextureToArray(rayMinTextureRef, rayCastData.d_rayIntervalSplatMinArray, channelDesc);
	//cudaBindTextureToArray(rayMaxTextureRef, rayCastData.d_rayIntervalSplatMaxArray, channelDesc);

	//printf("Ray casting render...\n");

	renderKernel<<<gridSize, blockSize>>>(hashData, rayCastData);

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
#endif
}

////////////////////////////////////////////////////////////////////////////////
//  Nicks render approach
////////////////////////////////////////////////////////////////////////////////

__global__ void clearDepthKernel(ftl::voxhash::HashData hashData, RayCastData rayCastData) 
{
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	const RayCastParams& rayCastParams = c_rayCastParams;

	if (x < rayCastParams.m_width && y < rayCastParams.m_height) {
		rayCastData.d_depth_i[y*rayCastParams.m_width+x] = 0x7FFFFFFF; //PINF;
		rayCastData.d_colors[y*rayCastParams.m_width+x] = make_uchar3(0,0,0);
	}
}

#define SDF_BLOCK_SIZE_PAD 9
#define SDF_BLOCK_BUFFER 1024  // > 9x9x9
#define SDF_DX 1
#define SDF_DY SDF_BLOCK_SIZE_PAD
#define SDF_DZ (SDF_BLOCK_SIZE_PAD*SDF_BLOCK_SIZE_PAD)

__device__
float frac(float val) {
	return (val - floorf(val));
}
__device__
float3 frac(const float3& val) {
		return make_float3(frac(val.x), frac(val.y), frac(val.z));
}

__host__ size_t nickSharedMem() {
	return sizeof(float)*SDF_BLOCK_BUFFER +
		sizeof(uchar)*SDF_BLOCK_BUFFER +
		sizeof(float)*SDF_BLOCK_BUFFER +
		sizeof(float3)*SDF_BLOCK_BUFFER;
}

/*__device__ void loadVoxel(const ftl::voxhash::HashData &hash, const int3 &vox, float *sdf, uint *weight, float3 *colour) {
	ftl::voxhash::Voxel &v = hashData.getVoxel(vox);
	*sdf = v.sdf;
	*weight = v.weight;
	*colour = v.color;
}*/

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

__device__ inline void trilinearInterp(const ftl::voxhash::HashData &hashData, const ftl::voxhash::Voxel *voxels, const uint *ix, const float3 &pos, float &depth, uchar3 &colour) {
	float3 colorFloat = make_float3(0.0f, 0.0f, 0.0f);
	const float3 weight = frac(hashData.worldToVirtualVoxelPosFloat(pos));  // Should be world position of ray, not voxel??
	float dist = 0.0f;
	dist+= (1.0f-weight.x)*(1.0f-weight.y)*(1.0f-weight.z)*voxels[ix[0]].sdf; colorFloat+= (1.0f-weight.x)*(1.0f-weight.y)*(1.0f-weight.z)*make_float3(voxels[ix[0]].color);
	dist+=	   weight.x *(1.0f-weight.y)*(1.0f-weight.z)*voxels[ix[1]].sdf; colorFloat+=	   weight.x *(1.0f-weight.y)*(1.0f-weight.z)*make_float3(voxels[ix[1]].color);
	dist+= (1.0f-weight.x)*	   weight.y *(1.0f-weight.z)*voxels[ix[2]].sdf; colorFloat+= (1.0f-weight.x)*	   weight.y *(1.0f-weight.z)*make_float3(voxels[ix[2]].color);
	dist+= (1.0f-weight.x)*(1.0f-weight.y)*	   weight.z *voxels[ix[3]].sdf; colorFloat+= (1.0f-weight.x)*(1.0f-weight.y)*	   weight.z *make_float3(voxels[ix[3]].color);
	dist+=	   weight.x *	   weight.y *(1.0f-weight.z)*voxels[ix[4]].sdf; colorFloat+=	   weight.x *	   weight.y *(1.0f-weight.z)*make_float3(voxels[ix[4]].color);
	dist+= (1.0f-weight.x)*	   weight.y *	   weight.z *voxels[ix[5]].sdf; colorFloat+= (1.0f-weight.x)*	   weight.y *	   weight.z *make_float3(voxels[ix[5]].color);
	dist+=	   weight.x *(1.0f-weight.y)*	   weight.z *voxels[ix[6]].sdf; colorFloat+=	   weight.x *(1.0f-weight.y)*	   weight.z *make_float3(voxels[ix[6]].color);
	dist+=	   weight.x *	   weight.y *	   weight.z *voxels[ix[7]].sdf; colorFloat+=	   weight.x *	   weight.y *	   weight.z *make_float3(voxels[ix[7]].color);
	depth = dist;
	colour = make_uchar3(colorFloat);
}

__global__ void nickRenderKernel(ftl::voxhash::HashData hashData, RayCastData rayCastData, RayCastParams params) {
	// TODO(Nick) Reduce bank conflicts by aligning these
	__shared__ ftl::voxhash::Voxel voxels[SDF_BLOCK_BUFFER];
	__shared__ ftl::voxhash::HashEntry blocks[8];

	const uint i = threadIdx.x;	//inside of an SDF block

	//TODO (Nick) Either don't use compactified or re-run compacitification using render cam frustrum
	if (i == 0) blocks[0] = hashData.d_hashCompactified[blockIdx.x];
	else if (i <= 7) blocks[i] = hashData.getHashEntryForSDFBlockPos(blockDelinear(blocks[0].pos, i));

	// Make sure all hash entries are cached
	__syncthreads();

	const int3 pi_base = hashData.SDFBlockToVirtualVoxelPos(blocks[0].pos);
	const int3 vp = make_int3(hashData.delinearizeVoxelIndex(i));
	const int3 pi = pi_base + vp;
	const uint j = plinVoxelPos(vp);  // Padded linear index
	const float3 worldPos = hashData.virtualVoxelPosToWorld(pi);

	// Load distances and colours into shared memory + padding
	const ftl::voxhash::Voxel &v = hashData.d_SDFBlocks[blocks[0].ptr + i];
	voxels[j] = v;

	// TODO (Nick) Coalesce memory better?
	uint imod = i & 0x7;
	bool v_do = imod < 3;

	if (v_do) {
		const uint v_a = (i >> 3) & 0x7;
		const uint v_c = (i >> 6);
		const uint v_b = (imod >= 1) ? v_c : v_a; 
		const int3 v_cache = make_int3(((imod == 0) ? 8 : v_a), ((imod == 1) ? 8 : v_b), ((imod == 2) ? 8 : v_c));
		const int3 v_ii = make_int3((imod == 0) ? 0 : v_a, (imod == 1) ? 0 : v_b, (imod == 2) ? 0 : v_c);
		const int v_block = blockLinear((imod == 0) ? 1 : 0, (imod == 1) ? 1 : 0, (imod == 2) ? 1 : 0);
		ftl::voxhash::Voxel &padVox = voxels[plinVoxelPos(v_cache)];
		const uint ii = hashData.linearizeVoxelPos(v_ii);
		if (blocks[v_block].ptr != ftl::voxhash::FREE_ENTRY) padVox = hashData.d_SDFBlocks[blocks[v_block].ptr + ii];
		else deleteVoxel(padVox);
	}

	/*if (vp.x == 7) {
		ftl::voxhash::Voxel &padVox = voxels[plinVoxelPos(make_int3(vp.x+1,vp.y,vp.z))];
		const uint ii = hashData.linearizeVoxelPos(make_int3(0,vp.y,vp.z));
		//padVox = hashData.getVoxel(make_int3(pi.x+1,pi.y,pi.z));
		if (blocks[blockLinear(1,0,0)].ptr != ftl::voxhash::FREE_ENTRY) padVox = hashData.d_SDFBlocks[blocks[blockLinear(1,0,0)].ptr + ii];
		else deleteVoxel(padVox);
	}
	if (vp.y == 7) {
		ftl::voxhash::Voxel &padVox = voxels[plinVoxelPos(make_int3(vp.x,vp.y+1,vp.z))];
		const uint ii = hashData.linearizeVoxelPos(make_int3(vp.x,0,vp.z));
		//padVox = hashData.getVoxel(make_int3(pi.x,pi.y+1,pi.z));
		if (blocks[blockLinear(0,1,0)].ptr != ftl::voxhash::FREE_ENTRY) padVox = hashData.d_SDFBlocks[blocks[blockLinear(0,1,0)].ptr + ii];
		else deleteVoxel(padVox);
	}
	if (vp.z == 7) {
		ftl::voxhash::Voxel &padVox = voxels[plinVoxelPos(make_int3(vp.x,vp.y,vp.z+1))];
		const uint ii = hashData.linearizeVoxelPos(make_int3(vp.x,vp.y,0));
		//padVox = hashData.getVoxel(make_int3(pi.x,pi.y,pi.z+1));
		if (blocks[blockLinear(0,0,1)].ptr != ftl::voxhash::FREE_ENTRY) padVox = hashData.d_SDFBlocks[blocks[blockLinear(0,0,1)].ptr + ii];
		else deleteVoxel(padVox);
	}*/

	// Indexes of the 8 neighbor voxels in one direction
	const uint ix[8] = {
		j, j+SDF_DX, j+SDF_DY, j+SDF_DZ, j+SDF_DX+SDF_DY, j+SDF_DY+SDF_DZ,
		j+SDF_DX+SDF_DZ, j+SDF_DX+SDF_DY+SDF_DZ
	};

	__syncthreads();

	// If any weight is 0, skip this voxel
	const bool missweight = voxels[ix[0]].weight == 0 || voxels[ix[1]].weight == 0 || voxels[ix[2]].weight == 0 ||
			voxels[ix[3]].weight == 0 || voxels[ix[4]].weight == 0 || voxels[ix[5]].weight == 0 ||
			voxels[ix[6]].weight == 0 || voxels[ix[7]].weight == 0;
	if (missweight) return;

	// Trilinear Interpolation (simple and fast)
	/*float3 colorFloat = make_float3(0.0f, 0.0f, 0.0f);
	const float3 weight = frac(hashData.worldToVirtualVoxelPosFloat(worldPos));  // Should be world position of ray, not voxel??
	float dist = 0.0f;
	dist+= (1.0f-weight.x)*(1.0f-weight.y)*(1.0f-weight.z)*voxels[ix[0]].sdf; colorFloat+= (1.0f-weight.x)*(1.0f-weight.y)*(1.0f-weight.z)*make_float3(voxels[ix[0]].color);
	dist+=	   weight.x *(1.0f-weight.y)*(1.0f-weight.z)*voxels[ix[1]].sdf; colorFloat+=	   weight.x *(1.0f-weight.y)*(1.0f-weight.z)*make_float3(voxels[ix[1]].color);
	dist+= (1.0f-weight.x)*	   weight.y *(1.0f-weight.z)*voxels[ix[2]].sdf; colorFloat+= (1.0f-weight.x)*	   weight.y *(1.0f-weight.z)*make_float3(voxels[ix[2]].color);
	dist+= (1.0f-weight.x)*(1.0f-weight.y)*	   weight.z *voxels[ix[3]].sdf; colorFloat+= (1.0f-weight.x)*(1.0f-weight.y)*	   weight.z *make_float3(voxels[ix[3]].color);
	dist+=	   weight.x *	   weight.y *(1.0f-weight.z)*voxels[ix[4]].sdf; colorFloat+=	   weight.x *	   weight.y *(1.0f-weight.z)*make_float3(voxels[ix[4]].color);
	dist+= (1.0f-weight.x)*	   weight.y *	   weight.z *voxels[ix[5]].sdf; colorFloat+= (1.0f-weight.x)*	   weight.y *	   weight.z *make_float3(voxels[ix[5]].color);
	dist+=	   weight.x *(1.0f-weight.y)*	   weight.z *voxels[ix[6]].sdf; colorFloat+=	   weight.x *(1.0f-weight.y)*	   weight.z *make_float3(voxels[ix[6]].color);
	dist+=	   weight.x *	   weight.y *	   weight.z *voxels[ix[7]].sdf; colorFloat+=	   weight.x *	   weight.y *	   weight.z *make_float3(voxels[ix[7]].color);

	// Must finish using colours before updating colours
	__syncthreads();

	//voxels[j].color = make_uchar3(colorFloat);
	//voxels[j].sdf = dist;

	// What happens if fitlered voxel is put back?
	//hashData.d_SDFBlocks[blocks[0].ptr + i] = voxels[j];

	//return;*/

	bool is_surface = false;
	// Identify surfaces through sign change. Since we only check in one direction
	// it is fine to check for any sign change?
#pragma unroll
	for (int u=0; u<=1; u++) {
		for (int v=0; v<=1; v++) {
			for (int w=0; w<=1; w++) {
				const int3 uvi = make_int3(vp.x+u,vp.y+v,vp.z+w);

				// Skip these cases since we didn't load voxels properly
				if (uvi.x == 8 && uvi.y == 8 || uvi.x == 8 && uvi.z == 8 || uvi.y == 8 && uvi.z == 8) continue;

				if (signbit(voxels[j].sdf) != signbit(voxels[plinVoxelPos(uvi)].sdf)) {
					is_surface = true;
					break;
				}
			}
		}
	}

	// Only for surface voxels, work out screen coordinates
	// TODO Could adjust weights, strengthen on surface, weaken otherwise??
	if (!is_surface) return;

	const float3 camPos = params.m_viewMatrix * worldPos;
	const float2 screenPosf = DepthCameraData::cameraToKinectScreenFloat(camPos);
	const uint2 screenPos = make_uint2(make_int2(screenPosf)); //  + make_float2(0.5f, 0.5f)

	/*if (screenPos.x < params.m_width && screenPos.y < params.m_height && 
			rayCastData.d_depth[(screenPos.y)*params.m_width+screenPos.x] > camPos.z) {
		rayCastData.d_depth[(screenPos.y)*params.m_width+screenPos.x] = camPos.z;
		rayCastData.d_colors[(screenPos.y)*params.m_width+screenPos.x] = voxels[j].color;
	}*/

	//return;

	// For this voxel in hash, get its screen position and check it is on screen
	// Convert depth map to int by x1000 and use atomicMin
	//const int pixsize = static_cast<int>(c_hashParams.m_virtualVoxelSize*c_depthCameraParams.fx/camPos.z)+1;
	int pixsizeX = 10;  // Max voxel pixels
	int pixsizeY = 10;

	for (int y=0; y<pixsizeY; y++) {
		for (int x=0; x<pixsizeX; x++) {
			// TODO(Nick) Within a window, check pixels that have same voxel id
			// Then trilinear interpolate between current voxel and neighbors.
			const float3 pixelWorldPos = params.m_viewMatrixInverse * DepthCameraData::kinectDepthToSkeleton(screenPos.x+x,screenPos.y+y, camPos.z);
			const float3 posInVoxel = (pixelWorldPos - worldPos) / make_float3(c_hashParams.m_virtualVoxelSize,c_hashParams.m_virtualVoxelSize,c_hashParams.m_virtualVoxelSize);

			if (posInVoxel.x >= 1.0f || posInVoxel.y >= 1.0f || posInVoxel.z >= 1.0f) {
				pixsizeX = x;
				continue;
			}

			/*float depth;
			uchar3 col;
			trilinearInterp(hashData, voxels, ix, pixelWorldPos, depth, col);*/
			int idepth = static_cast<int>(camPos.z * 100.0f);

			// TODO (Nick) MAKE THIS ATOMIC!!!!
			/*if (screenPos.x+x < params.m_width && screenPos.y+y < params.m_height && 
					rayCastData.d_depth_i[(screenPos.y+y)*params.m_width+screenPos.x+x] > idepth) {
				rayCastData.d_depth[(screenPos.y+y)*params.m_width+screenPos.x+x] = idepth;
				rayCastData.d_colors[(screenPos.y+y)*params.m_width+screenPos.x+x] = voxels[j].color;
			}*/

			if (screenPos.x+x < params.m_width && screenPos.y+y < params.m_height) {
				int index = (screenPos.y+y)*params.m_width+screenPos.x+x;
				if (rayCastData.d_depth_i[index] > idepth && atomicMin(&rayCastData.d_depth_i[index], idepth) != idepth) {
					//rayCastData.d_depth[index] = idepth;
					rayCastData.d_colors[index] = voxels[j].color;
				}
			}
		}
		if (pixsizeX == 0) break;
	}
}

extern "C" void nickRenderCUDA(const ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams, const RayCastData &rayCastData, const RayCastParams &params)
{
	const dim3 clear_gridSize((params.m_width + T_PER_BLOCK - 1)/T_PER_BLOCK, (params.m_height + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 clear_blockSize(T_PER_BLOCK, T_PER_BLOCK);

	clearDepthKernel<<<clear_gridSize, clear_blockSize>>>(hashData, rayCastData);

	const unsigned int threadsPerBlock = SDF_BLOCK_SIZE*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE;
	const dim3 gridSize(hashParams.m_numOccupiedBlocks, 1);
	const dim3 blockSize(threadsPerBlock, 1);

	if (hashParams.m_numOccupiedBlocks > 0) {	//this guard is important if there is no depth in the current frame (i.e., no blocks were allocated)
		nickRenderKernel << <gridSize, blockSize >> >(hashData, rayCastData, params);
	}

	cudaSafeCall( cudaGetLastError() );
	#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
	#endif
}


/////////////////////////////////////////////////////////////////////////
// ray interval splatting
/////////////////////////////////////////////////////////////////////////

__global__ void resetRayIntervalSplatKernel(RayCastData data) 
{
	uint idx = blockIdx.x + blockIdx.y * NUM_GROUPS_X;
	data.point_cloud_[idx] = make_float3(MINF);
}

extern "C" void resetRayIntervalSplatCUDA(RayCastData& data, const RayCastParams& params)
{
	const dim3 gridSize(NUM_GROUPS_X, (params.m_maxNumVertices + NUM_GROUPS_X - 1) / NUM_GROUPS_X, 1); // ! todo check if need third dimension?
	const dim3 blockSize(1, 1, 1);

	resetRayIntervalSplatKernel<<<gridSize, blockSize>>>(data);

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
#endif
}

__global__ void rayIntervalSplatKernel(ftl::voxhash::HashData hashData, DepthCameraData depthCameraData, RayCastData rayCastData, DepthCameraData cameraData) 
{
	uint idx = blockIdx.x + blockIdx.y * NUM_GROUPS_X;

	const ftl::voxhash::HashEntry& entry = hashData.d_hashCompactified[idx];
	if (entry.ptr != ftl::voxhash::FREE_ENTRY) {
		//if (!hashData.isSDFBlockInCameraFrustumApprox(entry.pos)) return;
		const RayCastParams &params = c_rayCastParams;
		const float4x4& viewMatrix = params.m_viewMatrix;

		float3 worldCurrentVoxel = hashData.SDFBlockToWorld(entry.pos);

		float3 MINV = worldCurrentVoxel - c_hashParams.m_virtualVoxelSize / 2.0f;

		float3 maxv = MINV+SDF_BLOCK_SIZE*c_hashParams.m_virtualVoxelSize;

		float3 proj000 = cameraData.cameraToKinectProj(viewMatrix * make_float3(MINV.x, MINV.y, MINV.z));
		float3 proj100 = cameraData.cameraToKinectProj(viewMatrix * make_float3(maxv.x, MINV.y, MINV.z));
		float3 proj010 = cameraData.cameraToKinectProj(viewMatrix * make_float3(MINV.x, maxv.y, MINV.z));
		float3 proj001 = cameraData.cameraToKinectProj(viewMatrix * make_float3(MINV.x, MINV.y, maxv.z));
		float3 proj110 = cameraData.cameraToKinectProj(viewMatrix * make_float3(maxv.x, maxv.y, MINV.z));
		float3 proj011 = cameraData.cameraToKinectProj(viewMatrix * make_float3(MINV.x, maxv.y, maxv.z));
		float3 proj101 = cameraData.cameraToKinectProj(viewMatrix * make_float3(maxv.x, MINV.y, maxv.z));
		float3 proj111 = cameraData.cameraToKinectProj(viewMatrix * make_float3(maxv.x, maxv.y, maxv.z));

		// Tree Reduction Min
		float3 min00 = fminf(proj000, proj100);
		float3 min01 = fminf(proj010, proj001);
		float3 min10 = fminf(proj110, proj011);
		float3 min11 = fminf(proj101, proj111);

		float3 min0 = fminf(min00, min01);
		float3 min1 = fminf(min10, min11);

		float3 minFinal = fminf(min0, min1);

		// Tree Reduction Max
		float3 max00 = fmaxf(proj000, proj100);
		float3 max01 = fmaxf(proj010, proj001);
		float3 max10 = fmaxf(proj110, proj011);
		float3 max11 = fmaxf(proj101, proj111);

		float3 max0 = fmaxf(max00, max01);
		float3 max1 = fmaxf(max10, max11);

		float3 maxFinal = fmaxf(max0, max1);

		float depth = maxFinal.z;
		if(params.m_splatMinimum == 1) {
			depth = minFinal.z;
		}
		float depthWorld = cameraData.kinectProjToCameraZ(depth);

		//uint addr = idx*4;
		//rayCastData.d_vertexBuffer[addr] = make_float4(maxFinal.x, minFinal.y, depth, depthWorld);
		//rayCastData.d_vertexBuffer[addr+1] = make_float4(minFinal.x, minFinal.y, depth, depthWorld);
		//rayCastData.d_vertexBuffer[addr+2] = make_float4(maxFinal.x, maxFinal.y, depth, depthWorld);
		//rayCastData.d_vertexBuffer[addr+3] = make_float4(minFinal.x, maxFinal.y, depth, depthWorld);

		// Note (Nick) : Changed to create point cloud instead of vertex.
		uint addr = idx;
		rayCastData.point_cloud_[addr] = make_float3(maxFinal.x, maxFinal.y, depth);
		//printf("Ray: %f\n", depth);

		/*uint addr = idx*6;
		rayCastData.d_vertexBuffer[addr] = make_float4(maxFinal.x, minFinal.y, depth, depthWorld);
		rayCastData.d_vertexBuffer[addr+1] = make_float4(minFinal.x, minFinal.y, depth, depthWorld);
		rayCastData.d_vertexBuffer[addr+2] = make_float4(maxFinal.x, maxFinal.y, depth, depthWorld);
		rayCastData.d_vertexBuffer[addr+3] = make_float4(minFinal.x, minFinal.y, depth, depthWorld);
		rayCastData.d_vertexBuffer[addr+4] = make_float4(maxFinal.x, maxFinal.y, depth, depthWorld);
		rayCastData.d_vertexBuffer[addr+5] = make_float4(minFinal.x, maxFinal.y, depth, depthWorld);*/
	}
}

extern "C" void rayIntervalSplatCUDA(const ftl::voxhash::HashData& hashData, const DepthCameraData& cameraData, const RayCastData &rayCastData, const RayCastParams &rayCastParams) 
{
	//printf("Ray casting...\n");
	const dim3 gridSize(NUM_GROUPS_X, (rayCastParams.m_numOccupiedSDFBlocks + NUM_GROUPS_X - 1) / NUM_GROUPS_X, 1);
	const dim3 blockSize(1, 1, 1);

	rayIntervalSplatKernel<<<gridSize, blockSize>>>(hashData, cameraData, rayCastData, cameraData);

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
#endif
}  
