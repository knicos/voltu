#ifndef _FTL_ILW_CUDA_HPP_
#define _FTL_ILW_CUDA_HPP_

#include <ftl/cuda_common.hpp>
#include <ftl/rgbd/camera.hpp>
#include <ftl/cuda_matrix_util.hpp>

namespace ftl {
namespace cuda {

struct ILWParams {
    float spatial_smooth;
    float colour_smooth;
    float cost_ratio;
    float threshold;
    uint flags;
};

static const uint kILWFlag_IgnoreBad = 0x0001;
static const uint kILWFlag_RestrictZ = 0x0002;
static const uint kILWFlag_SkipBadColour = 0x0004;
static const uint kILWFlag_ColourConfidenceOnly = 0x0008;

void correspondence_energy_vector(
    ftl::cuda::TextureObject<float4> &p1,
    ftl::cuda::TextureObject<float4> &p2,
    ftl::cuda::TextureObject<uchar4> &c1,
    ftl::cuda::TextureObject<uchar4> &c2,
    ftl::cuda::TextureObject<float4> &vout,
    ftl::cuda::TextureObject<float> &eout,
    float4x4 &pose1,
    float4x4 &pose1_inv,
    float4x4 &pose2,
    const ftl::rgbd::Camera &cam1,
    const ftl::rgbd::Camera &cam2,
    const ILWParams &params, int win,
    cudaStream_t stream
);

void move_points(
    ftl::cuda::TextureObject<float4> &p,
    ftl::cuda::TextureObject<float4> &v,
    const ftl::rgbd::Camera &camera,
    const float4x4 &pose,
    float rate,
    int radius,
    cudaStream_t stream
);

}
}

#endif  // _FTL_ILW_CUDA_HPP_
