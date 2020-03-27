#ifndef _FTL_OPTFLOW_CUDA_HPP_
#define _FTL_OPTFLOW_CUDA_HPP_

#include <ftl/cuda_common.hpp>
#include <ftl/rgbd/camera.hpp>

namespace ftl {
namespace cuda {

void show_optical_flow(const cv::cuda::GpuMat &optflow, const ftl::cuda::TextureObject<uchar4> &colour, float scale, cudaStream_t);

void disparity_from_flow(const cv::cuda::GpuMat &optflow1, const cv::cuda::GpuMat &optflow2, const ftl::cuda::TextureObject<short> &disparity, cudaStream_t stream);

void show_optical_flow_diff(const cv::cuda::GpuMat &optflow1, const cv::cuda::GpuMat &optflow2, const ftl::cuda::TextureObject<uchar4> &colour, float scale, const ftl::rgbd::Camera &cam, cudaStream_t stream);

}
}

#endif
