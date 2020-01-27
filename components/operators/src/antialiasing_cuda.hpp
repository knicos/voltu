#ifndef _FTL_CUDA_ANTIALIASING_HPP_
#define _FTL_CUDA_ANTIALIASING_HPP_

#include <ftl/cuda_common.hpp>

namespace ftl {
namespace cuda {

void fxaa(ftl::cuda::TextureObject<uchar4> &colour, cudaStream_t stream);
void fxaa(ftl::cuda::TextureObject<uchar4> &colour, ftl::cuda::TextureObject<float> &depth, float threshold, cudaStream_t stream);

}
}

#endif
