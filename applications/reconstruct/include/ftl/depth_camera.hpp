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
	cudaTextureObject_t colour;
	DepthCameraParams params;
	float4x4 pose;
	float4x4 poseInverse;
};

struct DepthCamera {

	///////////////
	// Host part //
	///////////////

	__host__
	DepthCamera() {
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
		depth_mat_ = new cv::cuda::GpuMat(params.m_imageHeight, params.m_imageWidth, CV_32FC1);
		colour_mat_ = new cv::cuda::GpuMat(params.m_imageHeight, params.m_imageWidth, CV_8UC4);
		depth_tex_ = new ftl::cuda::TextureObject<float>((cv::cuda::PtrStepSz<float>)*depth_mat_);
		colour_tex_ = new ftl::cuda::TextureObject<uchar4>((cv::cuda::PtrStepSz<uchar4>)*colour_mat_);
		data.depth = depth_tex_->cudaTexture();
		data.colour = colour_tex_->cudaTexture();
		data.params = params;
	}

	//__host__
	//void updateParams(const DepthCameraParams& params) {
	//	updateConstantDepthCameraParams(params);
	//}

	__host__
	void updateData(const cv::Mat &depth, const cv::Mat &rgb, cv::cuda::Stream &stream) {
		depth_mat_->upload(depth, stream);
		colour_mat_->upload(rgb, stream);
	}

	__host__
	void free() {
		if (depth_mat_) delete depth_mat_;
		if (colour_mat_) delete colour_mat_;
		delete depth_tex_;
		delete colour_tex_;
	}


	// TODO(Nick) Should not need to pass all these pointers to device
	cv::cuda::GpuMat *depth_mat_;
	cv::cuda::GpuMat *colour_mat_;
	ftl::cuda::TextureObject<float> *depth_tex_;
	ftl::cuda::TextureObject<uchar4> *colour_tex_;
	//cudaTextureObject_t depth_obj_;
	//cudaTextureObject_t colour_obj_;

	DepthCameraCUDA data;
};
}
}
