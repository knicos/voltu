#ifndef _FTL_CUDA_FILLING_HPP_
#define _FTL_CUDA_FILLING_HPP_

#include <ftl/rgbd/camera.hpp>
#include <ftl/cuda_common.hpp>

namespace ftl {
namespace cuda {

void scan_field_fill(
		ftl::cuda::TextureObject<float> &depth_in,
		ftl::cuda::TextureObject<float> &depth_out,
		ftl::cuda::TextureObject<float> &smoothing,
		float thresh,
		const ftl::rgbd::Camera &camera,
		cudaStream_t stream);

void filling_csr(
		ftl::cuda::TextureObject<uchar4> &region,
		ftl::cuda::TextureObject<float4> &normals_in,
		ftl::cuda::TextureObject<float> &depth_in,
		ftl::cuda::TextureObject<float> &depth_out,
		ftl::cuda::TextureObject<uchar4> &colour_in,
		const ftl::rgbd::Camera &camera,
		cudaStream_t stream);

}
}

#endif