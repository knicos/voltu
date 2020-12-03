/**
 * @file depth_convert_cuda.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#ifndef _FTL_CODECS_DEPTH_CONVERT_HPP_
#define _FTL_CODECS_DEPTH_CONVERT_HPP_

#include <ftl/cuda_common.hpp>

namespace ftl {
namespace cuda {

/*
 * See: Pece F., Kautz J., Weyrich T. 2011. Adapting standard video codecs for
 *      depth streaming. Joint Virtual Reality Conference of EGVE 2011 -
 *      The 17th Eurographics Symposium on Virtual Environments, EuroVR 2011 -
 *      The 8th EuroVR (INTUITION) Conference, , pp. 59-66.
 *
 */

/**
 * Convert a float depth image into a Yuv colour image (4 channel). In this
 * version, the colour data is packed together, not planar.
 * 
 * @param depth float 32 depth image
 * @param rgba Storage of same size as depth image for vuY(a) data output
 * @param maxdepth Used to normalise depth from 0 to maxdepth into 0 to 1.
 * 
 * @deprecated
 */
void depth_to_vuya(const cv::cuda::PtrStepSz<float> &depth, const cv::cuda::PtrStepSz<uchar4> &rgba, float maxdepth, cv::cuda::Stream &stream);

/**
 * Convert a float depth image into a 10bit NV12 colour image. The channels are
 * stored as 16bit, although only the upper 10bits are used. The format is
 * 4:2:0, so full resolution luminance but both chroma channels are packed to
 * cover 4 pixels. Chroma therefore has same width as depth but half the height.
 * 
 * @param depth float 32 depth image
 * @param luminance Luminance output.
 * @param chroma Both chroma outputs.
 * @param pitch Pitch in pixels of luma and chroma buffer.
 * @param maxdepth Used to normalise depth from 0 to maxdepth into 0 to 1.
 */
void depth_to_nv12_10(const cv::cuda::PtrStepSz<float> &depth, ushort* luminance, ushort* chroma, int pitch, float maxdepth, cv::cuda::Stream &stream);

/**
 * Convert a Yuv (4 channel) colour image to float depth image. Non-planar.
 * 
 * @param depth Output buffer for float depth.
 * @param rgba vuY(a) input (chroma first).
 * @param maxdepth Used to undo normalisation and scale result.
 * 
 * @deprecated
 */
void vuya_to_depth(const cv::cuda::PtrStepSz<float> &depth, const cv::cuda::PtrStepSz<ushort4> &rgba, float maxdepth, cv::cuda::Stream &stream);

/**
 * Convert NV12 10bit colour image to float depth.
 * @see depth_to_nv12_10
 * 
 * @param depth Output for float data.
 * @param luminance Luminance plane input, same resolution as depth.
 * @param chroma Both chroma channels in 4:2:0 plane.
 * @param maxdepth Used to undo normalisation and scale result.
 */
void vuya_to_depth(const cv::cuda::PtrStepSz<float> &depth, const cv::cuda::PtrStepSz<ushort> &luminance, const cv::cuda::PtrStepSz<ushort> &chroma, float maxdepth, cv::cuda::Stream &stream);

// TODO: (nick) Remove.
/** @deprecated and unused? */
void smooth_y(const cv::cuda::PtrStepSz<ushort4> &rgba, cv::cuda::Stream &stream);

/**
 * For lossless depth decode using NV12 input data (8bit). The input frame is
 * double width where the first half is the lower 8bits of the depth data and
 * the second half contains the upper 8bits. Therefore depth is stored 16bit
 * and is scaled by 1000.
 * 
 * @param src Raw NV12 buffer data
 * @param srcPitch Input pitch in bytes
 * @param dst Float depth output buffer
 * @param dstPitch Output pitch in pixels (4bpp)
 * @param width Image width
 * @param height Image height
 */
void nv12_to_float(const uint8_t* src, uint32_t srcPitch, float* dst, uint32_t dstPitch, uint32_t width, uint32_t height, cudaStream_t s);

// FIXME: Is the following correct? The code seems to be 8bit?
/**
 * Lossless encode of depth data to NV12. The encoding is 10bit and hence the
 * output is stored in 16bit form.
 * 
 * @see nv12_to_float
 * 
 * @param src Float depth image bufer
 * @param srcPitch Depth pitch in pixels (4bpp)
 * @param dst NV12 (16bit) output buffer.
 * @param dstPitch NV12 pitch in bytes.
 * @param width Image width.
 * @param height Image height.
 */
void float_to_nv12_16bit(const float* src, uint32_t srcPitch, uchar* dst, uint32_t dstPitch, uint32_t width, uint32_t height, cudaStream_t s);

}
}

// Taken from defunct NvPipe library.
template <class COLOR32>
void Nv12ToColor32(uint8_t *dpNv12, int nNv12Pitch, uint8_t *dpBgra, int nBgraPitch, int nWidth, int nHeight, int iMatrix = 0, cudaStream_t s=0);

#endif  // _FTL_CODECS_DEPTH_CONVERT_HPP_
