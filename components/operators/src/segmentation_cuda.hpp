#ifndef _FTL_CUDA_SEGMENTATION_HPP_
#define _FTL_CUDA_SEGMENTATION_HPP_

#include <ftl/cuda_common.hpp>

namespace ftl {
namespace cuda {

void support_region(
		ftl::cuda::TextureObject<uchar4> &colour,
		ftl::cuda::TextureObject<uchar4> &region,
		int tau, int v_max, int h_max,
		cudaStream_t stream);

void vis_support_region(
        ftl::cuda::TextureObject<uchar4> &colour,
        ftl::cuda::TextureObject<uchar4> &region,
        cudaStream_t stream);

}
}

#endif 
