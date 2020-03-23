#ifndef _FTL_CUDA_GT_HPP_
#define _FTL_CUDA_GT_HPP_

#include <ftl/cuda_common.hpp>
#include <ftl/rgbd/camera.hpp>

namespace ftl {
namespace cuda {

struct GTAnalysisData {
	int valid;			// number of pixels with valid pixels
	int missing;		// number of missing non-masked pixels
	int missing_masked;	// number of missing masked pixels
	int masked;			// number of pixels masked (in gt)

	int good;			// number of good pixels (valid value and error within min/max threshold)
	float err;			// sum of absolute error
	float err_sq;		// sum of squared error
};

void gt_analysis(
	ftl::cuda::TextureObject<uchar4> &colour,
	ftl::cuda::TextureObject<float> &depth,
	ftl::cuda::TextureObject<float> &gt,
	ftl::cuda::TextureObject<uchar> &mask,
	ftl::cuda::GTAnalysisData *out,
	const ftl::rgbd::Camera &cam,
	float t_min,
	float t_max,
	uchar4 colour_value,
	bool use_disparity,
	cudaStream_t stream
);

void gt_analysis(
	ftl::cuda::TextureObject<float> &depth,
	ftl::cuda::TextureObject<float> &gt,
	ftl::cuda::TextureObject<uchar> &mask,
	ftl::cuda::GTAnalysisData *out,
	const ftl::rgbd::Camera &cam,
	float t_min,
	float t_max,
	bool use_disparity,
	cudaStream_t stream
);

}
}

#endif