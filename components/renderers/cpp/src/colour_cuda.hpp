#ifndef _FTL_RENDER_COLOUR_CUDA_HPP_
#define _FTL_RENDER_COLOUR_CUDA_HPP_

#include <ftl/cuda_common.hpp>

namespace ftl {
namespace cuda {

template <typename T>
void lut(ftl::cuda::TextureObject<T> &in, ftl::cuda::TextureObject<uchar4> &out,
		const cv::cuda::PtrStepSz<uchar3> &lut, float minval, float maxval,
		bool invert, cudaStream_t stream);

void blend_alpha(
		ftl::cuda::TextureObject<uchar4> &in,
		ftl::cuda::TextureObject<uchar4> &out,
		float alpha, float beta,
		cudaStream_t stream);

}
}

#endif 