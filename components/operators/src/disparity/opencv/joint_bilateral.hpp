#ifndef _FTL_CUDA_OPENCV_BILATERAL_HPP_
#define _FTL_CUDA_OPENCV_BILATERAL_HPP_

#include <ftl/cuda_common.hpp>

namespace ftl {
namespace cuda {
	cv::Ptr<cv::cuda::DisparityBilateralFilter> createDisparityBilateralFilter(int ndisp, int radius, int iters);
}
}

#endif  // _FTL_CUDA_OPENCV_BILATERAL_HPP_
