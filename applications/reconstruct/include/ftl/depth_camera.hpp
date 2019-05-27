// From: https://github.com/niessner/VoxelHashing/blob/master/DepthSensingCUDA/Source/DepthCameraUtil.h

#pragma once

//#include <cutil_inline.h>
//#include <cutil_math.h>
#include <vector_types.h>
#include <cuda_runtime.h>

#include <ftl/cuda_matrix_util.hpp>
#include <ftl/cuda_common.hpp>

#include <ftl/depth_camera_params.hpp>


extern "C" void updateConstantDepthCameraParams(const DepthCameraParams& params);
extern __constant__ DepthCameraParams c_depthCameraParams;


struct DepthCameraData {

	///////////////
	// Host part //
	///////////////

	__device__ __host__
	DepthCameraData() {
		/*d_depthData = NULL;
		d_colorData = NULL;
		d_depthArray = NULL;
		d_colorArray = NULL;*/

		depth_mat_ = nullptr;
		colour_mat_ = nullptr;
		depth_tex_ = nullptr;
		colour_tex_ = nullptr;
	}

	__host__
	void alloc(const DepthCameraParams& params) { //! todo resizing???
		/*cudaSafeCall(cudaMalloc(&d_depthData, sizeof(float) * params.m_imageWidth * params.m_imageHeight));
		cudaSafeCall(cudaMalloc(&d_colorData, sizeof(float4) * params.m_imageWidth * params.m_imageHeight));

		h_depthChannelDesc = cudaCreateChannelDesc(32, 0, 0, 0, cudaChannelFormatKindFloat);
		cudaSafeCall(cudaMallocArray(&d_depthArray, &h_depthChannelDesc, params.m_imageWidth, params.m_imageHeight));
		h_colorChannelDesc = cudaCreateChannelDesc(32, 32, 32, 32, cudaChannelFormatKindFloat);
		cudaSafeCall(cudaMallocArray(&d_colorArray, &h_colorChannelDesc, params.m_imageWidth, params.m_imageHeight));*/

		std::cout << "Create texture objects: " << params.m_imageWidth << "," << params.m_imageHeight << std::endl;
		depth_mat_ = new cv::cuda::GpuMat(params.m_imageHeight, params.m_imageWidth, CV_32FC1);
		colour_mat_ = new cv::cuda::GpuMat(params.m_imageHeight, params.m_imageWidth, CV_8UC4);
		depth_tex_ = new ftl::cuda::TextureObject<float>((cv::cuda::PtrStepSz<float>)*depth_mat_);
		colour_tex_ = new ftl::cuda::TextureObject<uchar4>((cv::cuda::PtrStepSz<uchar4>)*colour_mat_);
		depth_obj_ = depth_tex_->cudaTexture();
		colour_obj_ = colour_tex_->cudaTexture();
	}

	__host__
	void updateParams(const DepthCameraParams& params) {
		updateConstantDepthCameraParams(params);
	}

	__host__
	void updateData(const cv::Mat &depth, const cv::Mat &rgb) {
		depth_mat_->upload(depth);
		colour_mat_->upload(rgb);
	}

	__host__
	void free() {
		/*if (d_depthData) cudaSafeCall(cudaFree(d_depthData));
		if (d_colorData) cudaSafeCall(cudaFree(d_colorData));
		if (d_depthArray) cudaSafeCall(cudaFreeArray(d_depthArray));
		if (d_colorArray) cudaSafeCall(cudaFreeArray(d_colorArray));*/

		/*d_depthData = NULL;
		d_colorData = NULL;
		d_depthArray = NULL;
		d_colorArray = NULL;*/

		if (depth_mat_) delete depth_mat_;
		if (colour_mat_) delete colour_mat_;
		delete depth_tex_;
		delete colour_tex_;
	}


	/////////////////
	// Device part //
	/////////////////

	static inline const DepthCameraParams& params() {
		return c_depthCameraParams;
	}

		///////////////////////////////////////////////////////////////
		// Camera to Screen
		///////////////////////////////////////////////////////////////

	__device__
	static inline float2 cameraToKinectScreenFloat(const float3& pos)	{
		//return make_float2(pos.x*c_depthCameraParams.fx/pos.z + c_depthCameraParams.mx, c_depthCameraParams.my - pos.y*c_depthCameraParams.fy/pos.z);
		return make_float2(
			pos.x*c_depthCameraParams.fx/pos.z + c_depthCameraParams.mx,			
			pos.y*c_depthCameraParams.fy/pos.z + c_depthCameraParams.my);
	}

	__device__
	static inline int2 cameraToKinectScreenInt(const float3& pos)	{
		float2 pImage = cameraToKinectScreenFloat(pos);
		return make_int2(pImage + make_float2(0.5f, 0.5f));
	}

	__device__
	static inline uint2 cameraToKinectScreen(const float3& pos)	{
		int2 p = cameraToKinectScreenInt(pos);
		return make_uint2(p.x, p.y);
	}

	__device__
	static inline float cameraToKinectProjZ(float z)	{
		return (z - c_depthCameraParams.m_sensorDepthWorldMin)/(c_depthCameraParams.m_sensorDepthWorldMax - c_depthCameraParams.m_sensorDepthWorldMin);
	}

	__device__
	static inline float3 cameraToKinectProj(const float3& pos) {
		float2 proj = cameraToKinectScreenFloat(pos);

		float3 pImage = make_float3(proj.x, proj.y, pos.z);

		pImage.x = (2.0f*pImage.x - (c_depthCameraParams.m_imageWidth- 1.0f))/(c_depthCameraParams.m_imageWidth- 1.0f);
		//pImage.y = (2.0f*pImage.y - (c_depthCameraParams.m_imageHeight-1.0f))/(c_depthCameraParams.m_imageHeight-1.0f);
		pImage.y = ((c_depthCameraParams.m_imageHeight-1.0f) - 2.0f*pImage.y)/(c_depthCameraParams.m_imageHeight-1.0f);
		pImage.z = cameraToKinectProjZ(pImage.z);

		return pImage;
	}

		///////////////////////////////////////////////////////////////
		// Screen to Camera (depth in meters)
		///////////////////////////////////////////////////////////////

	__device__
	static inline float3 kinectDepthToSkeleton(uint ux, uint uy, float depth)	{
		const float x = ((float)ux-c_depthCameraParams.mx) / c_depthCameraParams.fx;
		const float y = ((float)uy-c_depthCameraParams.my) / c_depthCameraParams.fy;
		//const float y = (c_depthCameraParams.my-(float)uy) / c_depthCameraParams.fy;
		return make_float3(depth*x, depth*y, depth);
	}

		///////////////////////////////////////////////////////////////
		// RenderScreen to Camera -- ATTENTION ASSUMES [1,0]-Z range!!!!
		///////////////////////////////////////////////////////////////

	__device__
	static inline float kinectProjToCameraZ(float z) {
		return z * (c_depthCameraParams.m_sensorDepthWorldMax - c_depthCameraParams.m_sensorDepthWorldMin) + c_depthCameraParams.m_sensorDepthWorldMin;
	}

	// z has to be in [0, 1]
	__device__
	static inline float3 kinectProjToCamera(uint ux, uint uy, float z)	{
		float fSkeletonZ = kinectProjToCameraZ(z);
		return kinectDepthToSkeleton(ux, uy, fSkeletonZ);
	}
	
	__device__
	static inline bool isInCameraFrustumApprox(const float4x4& viewMatrixInverse, const float3& pos) {
		float3 pCamera = viewMatrixInverse * pos;
		float3 pProj = cameraToKinectProj(pCamera);
		//pProj *= 1.5f;	//TODO THIS IS A HACK FIX IT :)
		pProj *= 0.95;
		return !(pProj.x < -1.0f || pProj.x > 1.0f || pProj.y < -1.0f || pProj.y > 1.0f || pProj.z < 0.0f || pProj.z > 1.0f);  
	}

	//float*		d_depthData;	//depth data of the current frame (in screen space):: TODO data allocation lives in RGBD Sensor
	//float4*		d_colorData;
	
	//uchar4*		d_colorData;	//color data of the current frame (in screen space):: TODO data allocation lives in RGBD Sensor

	cv::cuda::GpuMat *depth_mat_;
	cv::cuda::GpuMat *colour_mat_;
	ftl::cuda::TextureObject<float> *depth_tex_;
	ftl::cuda::TextureObject<uchar4> *colour_tex_;
	cudaTextureObject_t depth_obj_;
	cudaTextureObject_t colour_obj_;

	// cuda arrays for texture access
	/*cudaArray*	d_depthArray;
	cudaArray*	d_colorArray;
	cudaChannelFormatDesc h_depthChannelDesc;
	cudaChannelFormatDesc h_colorChannelDesc;*/
};
