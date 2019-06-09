// From: https://github.com/niessner/VoxelHashing/blob/master/DepthSensingCUDA/Source/CUDADepthCameraParams.h

//#include <cutil_inline.h>
//#include <cutil_math.h>
#include <vector_types.h>
#include <cuda_runtime.h>

#include <ftl/cuda_matrix_util.hpp>
#include <ftl/rgbd/camera.hpp>

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
};
