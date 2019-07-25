// From: https://github.com/niessner/VoxelHashing/blob/master/DepthSensingCUDA/Source/DepthCameraUtil.h

#pragma once

//#include <cutil_inline.h>
//#include <cutil_math.h>
#include <vector_types.h>
#include <cuda_runtime.h>

#include <ftl/cuda_matrix_util.hpp>
#include <ftl/cuda_common.hpp>

#include <ftl/depth_camera_params.hpp>

#define MAX_CAMERAS 20


//extern "C" void updateConstantDepthCameraParams(const DepthCameraParams& params);
//extern __constant__ DepthCameraParams c_depthCameraParams;

namespace ftl {
namespace voxhash {

struct __align__(16) DepthCameraCUDA {
	cudaTextureObject_t depth;
	cudaTextureObject_t depth2;
	cudaTextureObject_t points;
	cudaTextureObject_t colour;
	cudaTextureObject_t normal;
	DepthCameraParams params;
	float4x4 pose;
	float4x4 poseInverse;
};

struct DepthCamera {

	///////////////
	// Host part //
	///////////////

	__host__ DepthCamera();

	__host__ void alloc(const DepthCameraParams& params, bool withNormals=false);

	__host__ void updateData(const cv::Mat &depth, const cv::Mat &rgb, cv::cuda::Stream &stream);

	__host__ void free();

	__host__ void _computeNormals(cudaStream_t stream);

	ftl::cuda::TextureObject<float> *depth_tex_;
	ftl::cuda::TextureObject<int> *depth2_tex_;
	ftl::cuda::TextureObject<float4> *points_tex_;
	ftl::cuda::TextureObject<uchar4> *colour_tex_;
	ftl::cuda::TextureObject<float4> *normal_tex_;

	// This part is sent to device
	DepthCameraCUDA data;
};
}
}
