#ifndef _FTL_CUDA_GT_HPP_
#define _FTL_CUDA_GT_HPP_

#include <ftl/cuda_common.hpp>
#include <ftl/rgbd/camera.hpp>

namespace ftl {
namespace cuda {

struct GTAnalysisData {
	int invalid;		// Count of invalid (missing depth)
	int bad;			// Count bad (above x disparity error)
	float totalerror;	// Summed disparity error (of valid values)
	int masked;			// Count of pixels masked.
};

void gt_analysis(
	ftl::cuda::TextureObject<uchar4> &colour,
	ftl::cuda::TextureObject<float> &depth,
	ftl::cuda::TextureObject<float> &gt,
	ftl::cuda::GTAnalysisData *out,
	const ftl::rgbd::Camera &cam,
	float threshold,
	float outmax,
	cudaStream_t stream
);

void gt_analysis(
	ftl::cuda::TextureObject<float> &depth,
	ftl::cuda::TextureObject<float> &gt,
	ftl::cuda::GTAnalysisData *out,
	const ftl::rgbd::Camera &cam,
	float threshold,
	cudaStream_t stream
);

}
}

#endif 