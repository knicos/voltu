#ifndef _FTL_CODECS_DEPTH_CONVERT_HPP_
#define _FTL_CODECS_DEPTH_CONVERT_HPP_

#include <ftl/cuda_common.hpp>

namespace ftl {
namespace cuda {

void depth_to_vuya(const cv::cuda::PtrStepSz<float> &depth, const cv::cuda::PtrStepSz<uchar4> &rgba, float maxdepth, cv::cuda::Stream &stream);

void depth_to_nv12_10(const cv::cuda::PtrStepSz<float> &depth, ushort* luminance, ushort* chroma, int pitch, float maxdepth, cv::cuda::Stream &stream);

void vuya_to_depth(const cv::cuda::PtrStepSz<float> &depth, const cv::cuda::PtrStepSz<ushort4> &rgba, float maxdepth, cv::cuda::Stream &stream);

void vuya_to_depth(const cv::cuda::PtrStepSz<float> &depth, const cv::cuda::PtrStepSz<ushort> &luminance, const cv::cuda::PtrStepSz<ushort> &chroma, float maxdepth, cv::cuda::Stream &stream);

void smooth_y(const cv::cuda::PtrStepSz<ushort4> &rgba, cv::cuda::Stream &stream);

void nv12_to_float(const uint8_t* src, uint32_t srcPitch, float* dst, uint32_t dstPitch, uint32_t width, uint32_t height, cudaStream_t s);

void float_to_nv12_16bit(const float* src, uint32_t srcPitch, uchar* dst, uint32_t dstPitch, uint32_t width, uint32_t height, cudaStream_t s);

}
}

template <class COLOR32>
void Nv12ToColor32(uint8_t *dpNv12, int nNv12Pitch, uint8_t *dpBgra, int nBgraPitch, int nWidth, int nHeight, int iMatrix = 0, cudaStream_t s=0);

#endif  // _FTL_CODECS_DEPTH_CONVERT_HPP_
