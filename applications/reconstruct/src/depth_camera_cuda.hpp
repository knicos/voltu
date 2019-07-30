#ifndef _FTL_RECONSTRUCTION_CAMERA_CUDA_HPP_
#define _FTL_RECONSTRUCTION_CAMERA_CUDA_HPP_

#include <ftl/depth_camera.hpp>
#include <ftl/voxel_hash.hpp>
#include "splat_params.hpp"

namespace ftl {
namespace cuda {

void clear_depth(const TextureObject<float> &depth, cudaStream_t stream);
void clear_depth(const TextureObject<int> &depth, cudaStream_t stream);
void clear_points(const ftl::cuda::TextureObject<float4> &depth, cudaStream_t stream);
void clear_colour(const ftl::cuda::TextureObject<uchar4> &depth, cudaStream_t stream);

void median_filter(const ftl::cuda::TextureObject<int> &in, ftl::cuda::TextureObject<float> &out, cudaStream_t stream);

void int_to_float(const ftl::cuda::TextureObject<int> &in, ftl::cuda::TextureObject<float> &out, float scale, cudaStream_t stream);

void float_to_int(const ftl::cuda::TextureObject<float> &in, ftl::cuda::TextureObject<int> &out, float scale, cudaStream_t stream);

void mls_smooth(TextureObject<float4> &output, const ftl::voxhash::HashParams &hashParams, int numcams, int cam, cudaStream_t stream);

void mls_resample(const TextureObject<int> &depthin, const TextureObject<uchar4> &colourin, TextureObject<float> &depthout, const ftl::voxhash::HashParams &hashParams, int numcams, const ftl::render::SplatParams &params, cudaStream_t stream);

void hole_fill(const TextureObject<int> &depth_in, const TextureObject<float> &depth_out, const DepthCameraParams &params, cudaStream_t stream);

void point_cloud(ftl::cuda::TextureObject<float4> &output, const ftl::voxhash::DepthCameraCUDA &depthCameraData, cudaStream_t stream);

void compute_normals(const ftl::cuda::TextureObject<float> &depth, ftl::cuda::TextureObject<float4> &normals, const ftl::voxhash::DepthCameraCUDA &camera, cudaStream_t stream);

}
}

#endif  // _FTL_RECONSTRUCTION_CAMERA_CUDA_HPP_
