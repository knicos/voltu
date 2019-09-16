// From: https://github.com/niessner/VoxelHashing/blob/master/DepthSensingCUDA/Source/CUDADepthCameraParams.h

#pragma once

//#include <cutil_inline.h>
//#include <cutil_math.h>
//#include <cuda_runtime.h>

#include <ftl/cuda_matrix_util.hpp>
#include <ftl/rgbd/camera.hpp>

//#include <vector_types.h>

struct __align__(16) DepthCameraParams {
	float fx;
	float fy;
	float mx;
	float my;

	unsigned short m_imageWidth;
	unsigned short m_imageHeight;
	unsigned int flags;

	float m_sensorDepthWorldMin;
	float m_sensorDepthWorldMax;

		///////////////////////////////////////////////////////////////
		// Camera to Screen
		///////////////////////////////////////////////////////////////

	__device__
	inline float2 cameraToKinectScreenFloat(const float3& pos) const {
		//return make_float2(pos.x*c_depthCameraParams.fx/pos.z + c_depthCameraParams.mx, c_depthCameraParams.my - pos.y*c_depthCameraParams.fy/pos.z);
		return make_float2(
			pos.x*fx/pos.z + mx,			
			pos.y*fy/pos.z + my);
	}

	__device__
	inline int2 cameraToKinectScreenInt(const float3& pos) const {
		float2 pImage = cameraToKinectScreenFloat(pos);
		return make_int2(pImage + make_float2(0.5f, 0.5f));
	}

	__device__
	inline uint2 cameraToKinectScreen(const float3& pos) const {
		int2 p = cameraToKinectScreenInt(pos);
		return make_uint2(p.x, p.y);
	}

	__device__
	inline float cameraToKinectProjZ(float z) const {
		return (z - m_sensorDepthWorldMin)/(m_sensorDepthWorldMax - m_sensorDepthWorldMin);
	}

	__device__
	inline float3 cameraToKinectProj(const float3& pos) const {
		float2 proj = cameraToKinectScreenFloat(pos);

		float3 pImage = make_float3(proj.x, proj.y, pos.z);

		pImage.x = (2.0f*pImage.x - (m_imageWidth- 1.0f))/(m_imageWidth- 1.0f);
		//pImage.y = (2.0f*pImage.y - (c_depthCameraParams.m_imageHeight-1.0f))/(c_depthCameraParams.m_imageHeight-1.0f);
		pImage.y = ((m_imageHeight-1.0f) - 2.0f*pImage.y)/(m_imageHeight-1.0f);
		pImage.z = cameraToKinectProjZ(pImage.z);

		return pImage;
	}

		///////////////////////////////////////////////////////////////
		// Screen to Camera (depth in meters)
		///////////////////////////////////////////////////////////////

	__device__
	inline float3 kinectDepthToSkeleton(uint ux, uint uy, float depth) const {
		const float x = ((float)ux-mx) / fx;
		const float y = ((float)uy-my) / fy;
		//const float y = (c_depthCameraParams.my-(float)uy) / c_depthCameraParams.fy;
		return make_float3(depth*x, depth*y, depth);
	}

		///////////////////////////////////////////////////////////////
		// RenderScreen to Camera -- ATTENTION ASSUMES [1,0]-Z range!!!!
		///////////////////////////////////////////////////////////////

	__device__
	inline float kinectProjToCameraZ(float z) const {
		return z * (m_sensorDepthWorldMax - m_sensorDepthWorldMin) + m_sensorDepthWorldMin;
	}

	// z has to be in [0, 1]
	__device__
	inline float3 kinectProjToCamera(uint ux, uint uy, float z) const {
		float fSkeletonZ = kinectProjToCameraZ(z);
		return kinectDepthToSkeleton(ux, uy, fSkeletonZ);
	}
	
	__device__
	inline bool isInCameraFrustumApprox(const float4x4& viewMatrixInverse, const float3& pos) const {
		float3 pCamera = viewMatrixInverse * pos;
		float3 pProj = cameraToKinectProj(pCamera);
		//pProj *= 1.5f;	//TODO THIS IS A HACK FIX IT :)
		pProj *= 0.95;
		return !(pProj.x < -1.0f || pProj.x > 1.0f || pProj.y < -1.0f || pProj.y > 1.0f || pProj.z < 0.0f || pProj.z > 1.0f);  
	}
};
