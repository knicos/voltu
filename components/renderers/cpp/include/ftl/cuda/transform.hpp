#ifndef _FTL_CUDA_TRANSFORM_HPP_
#define _FTL_CUDA_TRANSFORM_HPP_

#include <ftl/cuda_common.hpp>

namespace ftl {
namespace cuda {

    template <typename T>
	void flip(
		ftl::cuda::TextureObject<T> &out,
		cudaStream_t stream);


    template <typename T>
	void flip(
		cv::cuda::GpuMat &out,
		cudaStream_t stream);

}
}

#endif 
