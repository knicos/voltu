#ifndef _FTL_RECONSTRUCTION_SPLAT_CUDA_HPP_
#define _FTL_RECONSTRUCTION_SPLAT_CUDA_HPP_

#include <ftl/cuda_common.hpp>
#include <ftl/render/splat_params.hpp>

namespace ftl {
namespace cuda {
    void dibr_merge(ftl::cuda::TextureObject<float4> &points, ftl::cuda::TextureObject<int> &depth, ftl::render::SplatParams params, cudaStream_t stream);
}
}

#endif  // _FTL_RECONSTRUCTION_SPLAT_CUDA_HPP_
