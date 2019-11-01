#ifndef _FTL_CUDA_SMOOTHING_HPP_
#define _FTL_CUDA_SMOOTHING_HPP_

#include <ftl/rgbd/camera.hpp>
#include <ftl/cuda_common.hpp>

namespace ftl {
namespace cuda {

void depth_smooth(
	ftl::cuda::TextureObject<float> &depth_in,
	ftl::cuda::TextureObject<uchar4> &colour_in,
	ftl::cuda::TextureObject<float> &depth_out,
	const ftl::rgbd::Camera &camera,
	int radius, float factor, float thresh, int iters,
	cudaStream_t stream);

void smoothing_factor(
	ftl::cuda::TextureObject<float> &depth_in,
	//ftl::cuda::TextureObject<float> &depth_tmp,
	ftl::cuda::TextureObject<float> &temp,
	//ftl::cuda::TextureObject<uchar4> &colour_in,
	ftl::cuda::TextureObject<float> &smoothing,
	float thresh,
	const ftl::rgbd::Camera &camera,
	cudaStream_t stream);

}
}

#endif  // _FTL_CUDA_SMOOTHING_HPP_
