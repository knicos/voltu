#ifndef _FTL_CODECS_DEPTH_CONVERT_HPP_
#define _FTL_CODECS_DEPTH_CONVERT_HPP_

#include <ftl/cuda_common.hpp>

namespace ftl {
namespace cuda {

void depth_to_vuya(const cv::cuda::PtrStepSz<float> &depth, const cv::cuda::PtrStepSz<uchar4> &rgba, float maxdepth, cv::cuda::Stream stream);

void vuya_to_depth(const cv::cuda::PtrStepSz<float> &depth, const cv::cuda::PtrStepSz<ushort4> &rgba, float maxdepth, cv::cuda::Stream stream);

void smooth_y(const cv::cuda::PtrStepSz<ushort4> &rgba, cv::cuda::Stream stream);

}
}

#endif  // _FTL_CODECS_DEPTH_CONVERT_HPP_
