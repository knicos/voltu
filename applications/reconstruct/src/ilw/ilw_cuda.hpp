#ifndef _FTL_ILW_CUDA_HPP_
#define _FTL_ILW_CUDA_HPP_

#include <ftl/cuda_common.hpp>
#include <ftl/rgbd/camera.hpp>
#include <ftl/cuda_matrix_util.hpp>
#include <ftl/cuda/mask.hpp>

namespace ftl {
namespace cuda {

struct ILWParams {
	float spatial_smooth;
	float colour_smooth;
	float fill_match;
	float fill_threshold;
	float match_threshold;
	float cost_ratio;
	float cost_threshold;
	float range;
	uint flags;
};

static const uint kILWFlag_IgnoreBad = 0x0001;
static const uint kILWFlag_RestrictZ = 0x0002;
static const uint kILWFlag_SkipBadColour = 0x0004;
static const uint kILWFlag_ColourConfidenceOnly = 0x0008;

void discontinuity(
	ftl::cuda::TextureObject<int> &mask_out,
	ftl::cuda::TextureObject<float> &depth,
	const cv::Size size,
	const double minDepth,
	const double maxDepth,
	uint discon, cudaStream_t stream
);

void mask_filter(
	ftl::cuda::TextureObject<float> &depth,
	ftl::cuda::TextureObject<int> &mask,
	cudaStream_t stream);

void preprocess_depth(
	ftl::cuda::TextureObject<float> &depth_in,
	ftl::cuda::TextureObject<float> &depth_out,
	ftl::cuda::TextureObject<uchar4> &colour,
	ftl::cuda::TextureObject<int> &mask,
	const ftl::rgbd::Camera &camera,
	const ILWParams &params,
	cudaStream_t stream
);

void correspondence(
	ftl::cuda::TextureObject<float> &d1,
	ftl::cuda::TextureObject<float> &d2,
	ftl::cuda::TextureObject<uchar4> &c1,
	ftl::cuda::TextureObject<uchar4> &c2,
	ftl::cuda::TextureObject<float> &dout,
	ftl::cuda::TextureObject<float> &conf,
	ftl::cuda::TextureObject<int> &mask,
	float4x4 &pose1,
	float4x4 &pose1_inv,
	float4x4 &pose2,
	const ftl::rgbd::Camera &cam1,
	const ftl::rgbd::Camera &cam2,
	const ILWParams &params, int win,
	cudaStream_t stream
);

void move_points(
	ftl::cuda::TextureObject<float> &d_old,
	ftl::cuda::TextureObject<float> &d_new,
	ftl::cuda::TextureObject<float> &conf,
	const ftl::rgbd::Camera &camera,
	const float4x4 &pose,
	const ILWParams &params,
	float rate,
	int radius,
	cudaStream_t stream
);

}
}

#endif  // _FTL_ILW_CUDA_HPP_
