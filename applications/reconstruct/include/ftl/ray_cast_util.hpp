#pragma once

#include <ftl/cuda_matrix_util.hpp>
#include <ftl/depth_camera.hpp>
#include <ftl/voxel_hash.hpp>

#include <ftl/ray_cast_params.hpp>

struct RayCastSample
{
	float sdf;
	float alpha;
	uint weight;
	//uint3 color;
};

#ifndef MINF
#define MINF asfloat(0xff800000)
#endif

extern __constant__ RayCastParams c_rayCastParams;
extern "C" void updateConstantRayCastParams(const RayCastParams& params);


struct RayCastData {

	///////////////
	// Host part //
	///////////////

	__device__ __host__
	RayCastData() {
		d_depth = NULL;
		d_depth3 = NULL;
		d_normals = NULL;
		d_colors = NULL;

		//d_vertexBuffer = NULL;
		point_cloud_ = nullptr;

		d_rayIntervalSplatMinArray = NULL;
		d_rayIntervalSplatMaxArray = NULL;
	}

#ifndef __CUDACC__
	__host__
	void allocate(const RayCastParams& params) {
		//point_cloud_ = new cv::cuda::GpuMat(params.m_width*params.m_height, 1, CV_32FC3);
		cudaSafeCall(cudaMalloc(&d_depth, sizeof(float) * params.m_width * params.m_height));
		cudaSafeCall(cudaMalloc(&d_depth3, sizeof(float3) * params.m_width * params.m_height));
		cudaSafeCall(cudaMalloc(&d_normals, sizeof(float4) * params.m_width * params.m_height));
		cudaSafeCall(cudaMalloc(&d_colors, sizeof(uchar3) * params.m_width * params.m_height));
		//cudaSafeCall(cudaMalloc(&point_cloud_, sizeof(float3) * params.m_width * params.m_height));
		//printf("Allocate ray cast data: %lld \n", (unsigned long long)point_cloud_);
	}

	__host__
	void updateParams(const RayCastParams& params) {
		updateConstantRayCastParams(params);
	}

	__host__ void download(int *depth, uchar3 *colours, const RayCastParams& params) const {
		//printf("Download: %d,%d\n", params.m_width, params.m_height);
		if (depth) cudaSafeCall(cudaMemcpy(depth, d_depth_i, sizeof(int) * params.m_width * params.m_height, cudaMemcpyDeviceToHost));
		if (colours) cudaSafeCall(cudaMemcpy(colours, d_colors, sizeof(uchar3) * params.m_width * params.m_height, cudaMemcpyDeviceToHost));
	}

	__host__
		void free() {
			//cudaFree(point_cloud_);
			cudaFree(d_depth);
			cudaFree(d_depth3);
			cudaFree(d_normals);
			cudaFree(d_colors);
	}
#endif

	/////////////////
	// Device part //
	/////////////////
#ifdef __CUDACC__

	__device__
		const RayCastParams& params() const {
			return c_rayCastParams;
	}

	__device__
	float frac(float val) const {
		return (val - floorf(val));
	}
	__device__
	float3 frac(const float3& val) const {
			return make_float3(frac(val.x), frac(val.y), frac(val.z));
	}
	
	__device__
	bool trilinearInterpolationSimpleFastFast(const ftl::voxhash::HashData& hash, const float3& pos, float& dist, uchar3& color) const {
		const float oSet = c_hashParams.m_virtualVoxelSize;
		const float3 posDual = pos-make_float3(oSet/2.0f, oSet/2.0f, oSet/2.0f);
		float3 weight = frac(hash.worldToVirtualVoxelPosFloat(pos));

		dist = 0.0f;
		float3 colorFloat = make_float3(0.0f, 0.0f, 0.0f);
		ftl::voxhash::Voxel v = hash.getVoxel(posDual+make_float3(0.0f, 0.0f, 0.0f)); if(v.weight == 0) return false; float3 vColor = make_float3(v.color.x, v.color.y, v.color.z); dist+= (1.0f-weight.x)*(1.0f-weight.y)*(1.0f-weight.z)*v.sdf; colorFloat+= (1.0f-weight.x)*(1.0f-weight.y)*(1.0f-weight.z)*vColor;
		      v = hash.getVoxel(posDual+make_float3(oSet, 0.0f, 0.0f)); if(v.weight == 0) return false;		   vColor = make_float3(v.color.x, v.color.y, v.color.z); dist+=	   weight.x *(1.0f-weight.y)*(1.0f-weight.z)*v.sdf; colorFloat+=	   weight.x *(1.0f-weight.y)*(1.0f-weight.z)*vColor;
		      v = hash.getVoxel(posDual+make_float3(0.0f, oSet, 0.0f)); if(v.weight == 0) return false;		   vColor = make_float3(v.color.x, v.color.y, v.color.z); dist+= (1.0f-weight.x)*	   weight.y *(1.0f-weight.z)*v.sdf; colorFloat+= (1.0f-weight.x)*	   weight.y *(1.0f-weight.z)*vColor;
		      v = hash.getVoxel(posDual+make_float3(0.0f, 0.0f, oSet)); if(v.weight == 0) return false;		   vColor = make_float3(v.color.x, v.color.y, v.color.z); dist+= (1.0f-weight.x)*(1.0f-weight.y)*	   weight.z *v.sdf; colorFloat+= (1.0f-weight.x)*(1.0f-weight.y)*	   weight.z *vColor;
		      v = hash.getVoxel(posDual+make_float3(oSet, oSet, 0.0f)); if(v.weight == 0) return false;		   vColor = make_float3(v.color.x, v.color.y, v.color.z); dist+=	   weight.x *	   weight.y *(1.0f-weight.z)*v.sdf; colorFloat+=	   weight.x *	   weight.y *(1.0f-weight.z)*vColor;
		      v = hash.getVoxel(posDual+make_float3(0.0f, oSet, oSet)); if(v.weight == 0) return false;		   vColor = make_float3(v.color.x, v.color.y, v.color.z); dist+= (1.0f-weight.x)*	   weight.y *	   weight.z *v.sdf; colorFloat+= (1.0f-weight.x)*	   weight.y *	   weight.z *vColor;
		      v = hash.getVoxel(posDual+make_float3(oSet, 0.0f, oSet)); if(v.weight == 0) return false;		   vColor = make_float3(v.color.x, v.color.y, v.color.z); dist+=	   weight.x *(1.0f-weight.y)*	   weight.z *v.sdf; colorFloat+=	   weight.x *(1.0f-weight.y)*	   weight.z *vColor;
		      v = hash.getVoxel(posDual+make_float3(oSet, oSet, oSet)); if(v.weight == 0) return false;		   vColor = make_float3(v.color.x, v.color.y, v.color.z); dist+=	   weight.x *	   weight.y *	   weight.z *v.sdf; colorFloat+=	   weight.x *	   weight.y *	   weight.z *vColor;

		color = make_uchar3(colorFloat.x, colorFloat.y, colorFloat.z);//v.color;
		
		return true;
	}
	//__device__
	//bool trilinearInterpolationSimpleFastFast(const HashData& hash, const float3& pos, float& dist, uchar3& color) const {
	//	const float oSet = c_hashParams.m_virtualVoxelSize;
	//	const float3 posDual = pos-make_float3(oSet/2.0f, oSet/2.0f, oSet/2.0f);
	//	float3 weight = frac(hash.worldToVirtualVoxelPosFloat(pos));

	//	dist = 0.0f;
	//	Voxel v = hash.getVoxel(posDual+make_float3(0.0f, 0.0f, 0.0f)); if(v.weight == 0) return false; dist+= (1.0f-weight.x)*(1.0f-weight.y)*(1.0f-weight.z)*v.sdf;
	//	v = hash.getVoxel(posDual+make_float3(oSet, 0.0f, 0.0f)); if(v.weight == 0) return false;		dist+=	   weight.x *(1.0f-weight.y)*(1.0f-weight.z)*v.sdf;
	//	v = hash.getVoxel(posDual+make_float3(0.0f, oSet, 0.0f)); if(v.weight == 0) return false;		dist+= (1.0f-weight.x)*	   weight.y *(1.0f-weight.z)*v.sdf;
	//	v = hash.getVoxel(posDual+make_float3(0.0f, 0.0f, oSet)); if(v.weight == 0) return false;		dist+= (1.0f-weight.x)*(1.0f-weight.y)*	   weight.z *v.sdf;
	//	v = hash.getVoxel(posDual+make_float3(oSet, oSet, 0.0f)); if(v.weight == 0) return false;		dist+=	   weight.x *	   weight.y *(1.0f-weight.z)*v.sdf;
	//	v = hash.getVoxel(posDual+make_float3(0.0f, oSet, oSet)); if(v.weight == 0) return false;		dist+= (1.0f-weight.x)*	   weight.y *	   weight.z *v.sdf;
	//	v = hash.getVoxel(posDual+make_float3(oSet, 0.0f, oSet)); if(v.weight == 0) return false;		dist+=	   weight.x *(1.0f-weight.y)*	   weight.z *v.sdf;
	//	v = hash.getVoxel(posDual+make_float3(oSet, oSet, oSet)); if(v.weight == 0) return false;		dist+=	   weight.x *	   weight.y *	   weight.z *v.sdf;

	//	color = v.color;

	//	return true;
	//}


	__device__
	float findIntersectionLinear(float tNear, float tFar, float dNear, float dFar) const
	{
		return tNear+(dNear/(dNear-dFar))*(tFar-tNear);
	}
	
	static const unsigned int nIterationsBisection = 3;
	
	// d0 near, d1 far
	__device__
		bool findIntersectionBisection(const ftl::voxhash::HashData& hash, const float3& worldCamPos, const float3& worldDir, float d0, float r0, float d1, float r1, float& alpha, uchar3& color) const
	{
		float a = r0; float aDist = d0;
		float b = r1; float bDist = d1;
		float c = 0.0f;

#pragma unroll 1
		for(uint i = 0; i<nIterationsBisection; i++)
		{
			c = findIntersectionLinear(a, b, aDist, bDist);

			float cDist;
			if(!trilinearInterpolationSimpleFastFast(hash, worldCamPos+c*worldDir, cDist, color)) return false;

			if(aDist*cDist > 0.0) { a = c; aDist = cDist; }
			else { b = c; bDist = cDist; }
		}

		alpha = c;

		return true;
	}
	
	
	__device__
	float3 gradientForPoint(const ftl::voxhash::HashData& hash, const float3& pos) const
	{
		const float voxelSize = c_hashParams.m_virtualVoxelSize;
		float3 offset = make_float3(voxelSize, voxelSize, voxelSize);

		float distp00; uchar3 colorp00; trilinearInterpolationSimpleFastFast(hash, pos-make_float3(0.5f*offset.x, 0.0f, 0.0f), distp00, colorp00);
		float dist0p0; uchar3 color0p0; trilinearInterpolationSimpleFastFast(hash, pos-make_float3(0.0f, 0.5f*offset.y, 0.0f), dist0p0, color0p0);
		float dist00p; uchar3 color00p; trilinearInterpolationSimpleFastFast(hash, pos-make_float3(0.0f, 0.0f, 0.5f*offset.z), dist00p, color00p);

		float dist100; uchar3 color100; trilinearInterpolationSimpleFastFast(hash, pos+make_float3(0.5f*offset.x, 0.0f, 0.0f), dist100, color100);
		float dist010; uchar3 color010; trilinearInterpolationSimpleFastFast(hash, pos+make_float3(0.0f, 0.5f*offset.y, 0.0f), dist010, color010);
		float dist001; uchar3 color001; trilinearInterpolationSimpleFastFast(hash, pos+make_float3(0.0f, 0.0f, 0.5f*offset.z), dist001, color001);

		float3 grad = make_float3((distp00-dist100)/offset.x, (dist0p0-dist010)/offset.y, (dist00p-dist001)/offset.z);

		float l = length(grad);
		if(l == 0.0f) {
			return make_float3(0.0f, 0.0f, 0.0f);
		}

		return -grad/l;
	}

	__device__
	void traverseCoarseGridSimpleSampleAll(const ftl::voxhash::HashData& hash, const DepthCameraData& cameraData, const float3& worldCamPos, const float3& worldDir, const float3& camDir, const int3& dTid, float minInterval, float maxInterval) const
	{
		const RayCastParams& rayCastParams = c_rayCastParams;

		// Last Sample
		RayCastSample lastSample; lastSample.sdf = 0.0f; lastSample.alpha = 0.0f; lastSample.weight = 0; // lastSample.color = int3(0, 0, 0);
		const float depthToRayLength = 1.0f/camDir.z; // scale factor to convert from depth to ray length
		
		float rayCurrent = depthToRayLength * max(rayCastParams.m_minDepth, minInterval);	// Convert depth to raylength
		float rayEnd = depthToRayLength * min(rayCastParams.m_maxDepth, maxInterval);		// Convert depth to raylength
		//float rayCurrent = depthToRayLength * rayCastParams.m_minDepth;	// Convert depth to raylength
		//float rayEnd = depthToRayLength * rayCastParams.m_maxDepth;		// Convert depth to raylength

		//printf("MINMAX: %f,%f\n", minInterval, maxInterval);

#pragma unroll 1
		while(rayCurrent < rayEnd)
		{
			float3 currentPosWorld = worldCamPos+rayCurrent*worldDir;
			float dist;	uchar3 color;

			//printf("RAY LOOP: %f,%f,%f\n", currentPosWorld.x, currentPosWorld.y, currentPosWorld.z);

			if(trilinearInterpolationSimpleFastFast(hash, currentPosWorld, dist, color))
			{
				if(lastSample.weight > 0 && lastSample.sdf > 0.0f && dist < 0.0f) // current sample is always valid here 
				{

					float alpha; // = findIntersectionLinear(lastSample.alpha, rayCurrent, lastSample.sdf, dist);
					uchar3 color2;
					bool b = findIntersectionBisection(hash, worldCamPos, worldDir, lastSample.sdf, lastSample.alpha, dist, rayCurrent, alpha, color2);
					
					float3 currentIso = worldCamPos+alpha*worldDir;
					if(b && abs(lastSample.sdf - dist) < rayCastParams.m_thresSampleDist)
					{
						if(abs(dist) < rayCastParams.m_thresDist)
						{
							float depth = alpha / depthToRayLength; // Convert ray length to depth depthToRayLength

							d_depth[dTid.y*rayCastParams.m_width+dTid.x] = depth;
							d_depth3[dTid.y*rayCastParams.m_width+dTid.x] = cameraData.kinectDepthToSkeleton(dTid.x, dTid.y, depth);
							d_colors[dTid.y*rayCastParams.m_width+dTid.x] = make_uchar3(color2.x, color2.y, color2.z);

							if(rayCastParams.m_useGradients)
							{
								float3 normal = -gradientForPoint(hash, currentIso);
								float4 n = rayCastParams.m_viewMatrix * make_float4(normal, 0.0f);
								d_normals[dTid.y*rayCastParams.m_width+dTid.x] = make_float4(n.x, n.y, n.z, 1.0f);
							}

							return;
						}
					}
				}

				lastSample.sdf = dist;
				lastSample.alpha = rayCurrent;
				// lastSample.color = color;
				lastSample.weight = 1;
				rayCurrent += rayCastParams.m_rayIncrement;
			} else {
				lastSample.weight = 0;
				rayCurrent += rayCastParams.m_rayIncrement;
			}

			
		}
		
	}

#endif // __CUDACC__

	union {
	float*  d_depth;
	int* d_depth_i;
	};
	float3* d_depth3;
	float4* d_normals;
	uchar3* d_colors;

	//float4* d_vertexBuffer; // ray interval splatting triangles, mapped from directx (memory lives there)
	float3 *point_cloud_;

	cudaArray* d_rayIntervalSplatMinArray;
	cudaArray* d_rayIntervalSplatMaxArray;
};