#ifndef _FTL_RECONSTRUCTION_SPLAT_CUDA_HPP_
#define _FTL_RECONSTRUCTION_SPLAT_CUDA_HPP_

#include <ftl/cuda_common.hpp>
#include <ftl/render/splat_params.hpp>

namespace ftl {
namespace cuda {
    void dibr_merge(ftl::cuda::TextureObject<float4> &points, ftl::cuda::TextureObject<int> &depth, ftl::render::SplatParams params, cudaStream_t stream);

    void dibr_attribute(
        ftl::cuda::TextureObject<uchar4> &colour_in,    // Original colour image
        ftl::cuda::TextureObject<float4> &points,       // Original 3D points
        ftl::cuda::TextureObject<int> &depth_in,        // Virtual depth map
        ftl::cuda::TextureObject<float4> &colour_out,   // Accumulated output
        //TextureObject<float4> normal_out,
        ftl::cuda::TextureObject<float> &contrib_out,
        ftl::render::SplatParams &params, cudaStream_t stream);

    void dibr_normalise(
        ftl::cuda::TextureObject<float4> &colour_in,
        ftl::cuda::TextureObject<uchar4> &colour_out,
        ftl::cuda::TextureObject<float> &contribs,
        cudaStream_t stream);
}
}

#endif  // _FTL_RECONSTRUCTION_SPLAT_CUDA_HPP_
