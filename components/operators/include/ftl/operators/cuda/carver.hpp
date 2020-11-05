#ifndef _FTL_CUDA_CARVER_HPP_
#define _FTL_CUDA_CARVER_HPP_

#include <ftl/cuda_common.hpp>
#include <ftl/cuda_matrix_util.hpp>
#include <ftl/rgbd/camera.hpp>

namespace ftl {
namespace cuda {

/**
 * Carve `in` using `ref` as visibility reference.
 */
void depth_carve(
	cv::cuda::GpuMat &in,
	const cv::cuda::GpuMat &ref,
	//const cv::cuda::GpuMat &in_colour,
	//const cv::cuda::GpuMat &ref_colour,
	//cv::cuda::GpuMat &colour_scale,
	const float4x4 &transform,
	const ftl::rgbd::Camera &incam,
	const ftl::rgbd::Camera &refcam,
	cudaStream_t stream);

void apply_colour_scaling(
	const cv::cuda::GpuMat &scale,
	cv::cuda::GpuMat &colour,
	int radius,
	cudaStream_t stream);

}
}

#endif