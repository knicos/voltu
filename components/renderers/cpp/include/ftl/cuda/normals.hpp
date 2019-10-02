#ifndef _FTL_CUDA_NORMALS_HPP_
#define _FTL_CUDA_NORMALS_HPP_

#include <ftl/cuda_common.hpp>
#include <ftl/rgbd/camera.hpp>
#include <ftl/cuda_matrix_util.hpp>

namespace ftl {
namespace cuda {

void normals(ftl::cuda::TextureObject<float4> &output,
        ftl::cuda::TextureObject<float4> &temp,
        ftl::cuda::TextureObject<float4> &input, cudaStream_t stream);

void normal_visualise(ftl::cuda::TextureObject<float4> &norm,
        ftl::cuda::TextureObject<float> &output,
        const ftl::rgbd::Camera &camera, const float4x4 &pose,
        cudaStream_t stream);

}
}

#endif  // _FTL_CUDA_NORMALS_HPP_
