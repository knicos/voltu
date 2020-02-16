#ifndef _FTL_OPERATORS_WEIGHTING_CUDA_HPP_
#define _FTL_OPERATORS_WEIGHTING_CUDA_HPP_

#include <ftl/operators/mask_cuda.hpp>

namespace ftl {
namespace cuda {

struct PixelWeightingParameters {
	float depthCoef;
	float disconThresh;
	float noiseThresh;
	float areaMax;
	bool normals;
	bool depth;
	bool colour;
	bool noise;
};

void pixel_weighting(
		ftl::cuda::TextureObject<short> &weight_out,
		ftl::cuda::TextureObject<ftl::cuda::Mask::type> &mask_out,
		ftl::cuda::TextureObject<uchar4> &support,
		ftl::cuda::TextureObject<float> &depth,
		const ftl::rgbd::Camera &camera,
		const cv::Size size, const ftl::cuda::PixelWeightingParameters &params,
		cudaStream_t stream);

void pixel_weighting(
		ftl::cuda::TextureObject<short> &weight_out,
		ftl::cuda::TextureObject<ftl::cuda::Mask::type> &mask_out,
		ftl::cuda::TextureObject<half4> &normals_out,
		ftl::cuda::TextureObject<uchar4> &support,
		ftl::cuda::TextureObject<float> &depth,
		const ftl::rgbd::Camera &camera,
		const cv::Size size, const ftl::cuda::PixelWeightingParameters &params,
		cudaStream_t stream);

void cull_weight(
		ftl::cuda::TextureObject<short> &weights,
		ftl::cuda::TextureObject<float> &depth,
		float thresh, cudaStream_t stream);

void degrade_mask(
		ftl::cuda::TextureObject<uint8_t> &mask,
		ftl::cuda::TextureObject<short> &weights,
		uint8_t id,
		unsigned int radius,
		bool invert,
		cudaStream_t stream);

}
}

#endif
