#ifndef _FTL_CUDA_MLS_HPP_
#define _FTL_CUDA_MLS_HPP_

#include <ftl/rgbd/camera.hpp>
#include <ftl/cuda_common.hpp>

namespace ftl {
namespace cuda {

/**
 * Basic Moving Least Squares smoothing on a single depth map. Outputs boths
 * normals and depth values. This is a single iteration of an algorithm that
 * should iterate 2-3 times. Normals can use some basic estimation as initial
 * input since they are smoothed by MLS.
 * 
 * @param normals_in 4 channel 16-bit float (half).
 * @param normals_out 4 channel 16-bit float (half).
 * @param depth_in 1 channel 32-bit float
 * @param depth_out 1 channel 32-bit float
 * @param smoothing Gaussian radius in depth units
 * @param radius Window radius for smoothing
 * @param camera Camera intrinsics
 * @param stream Optional CUDA stream
 */
void mls_smooth(
	const cv::cuda::GpuMat &normals_in,
	cv::cuda::GpuMat &normals_out,
	const cv::cuda::GpuMat &depth_in,
	cv::cuda::GpuMat &depth_out,
	float smoothing,
	int radius,
	const ftl::rgbd::Camera &camera,
	cudaStream_t stream);

/**
 * Basic Moving Least Squares smoothing on a single depth map. Outputs just the
 * smoothed normals. This is a single iteration of an algorithm that should
 * iterate 2-3 times.
 * 
 * @param normals_in 4 channel 16-bit float (half).
 * @param normals_out 4 channel 16-bit float (half).
 * @param depth_in 1 channel 32-bit float
 * @param smoothing Gaussian radius in depth units
 * @param radius Window radius for smoothing
 * @param camera Camera intrinsics
 * @param stream Optional CUDA stream
 */
void mls_smooth(
	const cv::cuda::GpuMat &normals_in,
	cv::cuda::GpuMat &normals_out,
	const cv::cuda::GpuMat &depth_in,
	float smoothing,
	int radius,
	const ftl::rgbd::Camera &camera,
	cudaStream_t stream);

}
}

#endif  // _FTL_CUDA_MLS_HPP_
