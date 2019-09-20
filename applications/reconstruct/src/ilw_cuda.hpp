#ifndef _FTL_ILW_CUDA_HPP_
#define _FTL_ILW_CUDA_HPP_

#include <ftl/cuda_common.hpp>
#include <ftl/rgbd/camera.hpp>
#include <ftl/cuda_matrix_util.hpp>

namespace ftl {
namespace cuda {

void correspondence_energy_vector(
    ftl::cuda::TextureObject<float4> &p1,
    ftl::cuda::TextureObject<float4> &p2,
    ftl::cuda::TextureObject<uchar4> &c1,
    ftl::cuda::TextureObject<uchar4> &c2,
    ftl::cuda::TextureObject<float4> &vout,
    ftl::cuda::TextureObject<float> &eout,
    float4x4 &pose2,
    const ftl::rgbd::Camera &cam2,
    cudaStream_t stream
);

}
}

#endif  // _FTL_ILW_CUDA_HPP_
