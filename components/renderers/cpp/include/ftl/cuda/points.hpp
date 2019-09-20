#ifndef _FTL_CUDA_POINTS_HPP_
#define _FTL_CUDA_POINTS_HPP_

#include <ftl/cuda_common.hpp>
#include <ftl/rgbd/camera.hpp>
#include <ftl/cuda_matrix_util.hpp>

namespace ftl {
namespace cuda {

void point_cloud(ftl::cuda::TextureObject<float4> &output, ftl::cuda::TextureObject<float> &depth, const ftl::rgbd::Camera &params, const float4x4 &pose, cudaStream_t stream);

}
}

#endif  // _FTL_CUDA_POINTS_HPP_
