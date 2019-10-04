#ifndef _FTL_CUDA_NORMALS_HPP_
#define _FTL_CUDA_NORMALS_HPP_

#include <ftl/cuda_common.hpp>
#include <ftl/rgbd/camera.hpp>
#include <ftl/cuda_matrix_util.hpp>

namespace ftl {
namespace cuda {

void normals(ftl::cuda::TextureObject<float4> &output,
        ftl::cuda::TextureObject<float4> &temp,
        ftl::cuda::TextureObject<float4> &input,
        const ftl::rgbd::Camera &camera,
        const float3x3 &pose, cudaStream_t stream);

void normal_visualise(ftl::cuda::TextureObject<float4> &norm,
        ftl::cuda::TextureObject<uchar4> &output,
        const float3 &light, const uchar4 &diffuse, const uchar4 &ambient,
        cudaStream_t stream);

void normal_filter(ftl::cuda::TextureObject<float4> &norm,
        ftl::cuda::TextureObject<float4> &points,
        const ftl::rgbd::Camera &camera, const float4x4 &pose,
        float thresh, cudaStream_t stream);

}
}

#endif  // _FTL_CUDA_NORMALS_HPP_
