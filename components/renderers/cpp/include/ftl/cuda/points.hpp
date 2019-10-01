#ifndef _FTL_CUDA_POINTS_HPP_
#define _FTL_CUDA_POINTS_HPP_

#include <ftl/cuda_common.hpp>
#include <ftl/rgbd/camera.hpp>
#include <ftl/cuda_matrix_util.hpp>

namespace ftl {
namespace cuda {

struct ClipSpace {
	float4x4 origin;
	float3 size;
};

void point_cloud(ftl::cuda::TextureObject<float4> &output,
		ftl::cuda::TextureObject<float> &depth,
		const ftl::rgbd::Camera &params,
		const float4x4 &pose, cudaStream_t stream);

void clipping(ftl::cuda::TextureObject<float4> &points,
		const ClipSpace &clip, cudaStream_t stream);

void point_cloud(ftl::cuda::TextureObject<float> &output, ftl::cuda::TextureObject<float4> &points, const ftl::rgbd::Camera &params, const float4x4 &poseinv, cudaStream_t stream);

void world_to_cam(ftl::cuda::TextureObject<float4> &points, const float4x4 &poseinv, cudaStream_t stream);

void cam_to_world(ftl::cuda::TextureObject<float4> &points, const float4x4 &pose, cudaStream_t stream);

}
}

#endif  // _FTL_CUDA_POINTS_HPP_
