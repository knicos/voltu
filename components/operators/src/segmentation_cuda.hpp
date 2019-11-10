#ifndef _FTL_CUDA_SEGMENTATION_HPP_
#define _FTL_CUDA_SEGMENTATION_HPP_

#include <ftl/cuda_common.hpp>

namespace ftl {
namespace cuda {

void support_region(
		ftl::cuda::TextureObject<uchar4> &colour,
		ftl::cuda::TextureObject<uchar4> &region,
		float tau, int v_max, int h_max,
		cudaStream_t stream);

void support_region(
		ftl::cuda::TextureObject<float> &depth,
		ftl::cuda::TextureObject<uchar4> &region,
		float tau, int v_max, int h_max,
		cudaStream_t stream);

void vis_support_region(
        ftl::cuda::TextureObject<uchar4> &colour,
        ftl::cuda::TextureObject<uchar4> &region,
		uchar4 bar_colour,
		uchar4 area_colour,
		int ox, int oy, int dx, int dy,
        cudaStream_t stream);

void vis_bad_region(
		ftl::cuda::TextureObject<uchar4> &colour,
		ftl::cuda::TextureObject<float> &depth,
		ftl::cuda::TextureObject<uchar4> &region,
		ftl::cuda::TextureObject<uchar4> &dregion,
        cudaStream_t stream);


}
}

#endif 
