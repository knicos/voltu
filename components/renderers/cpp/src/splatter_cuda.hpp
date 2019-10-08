#ifndef _FTL_RECONSTRUCTION_SPLAT_CUDA_HPP_
#define _FTL_RECONSTRUCTION_SPLAT_CUDA_HPP_

#include <ftl/cuda_common.hpp>
#include <ftl/render/splat_params.hpp>

namespace ftl {
namespace cuda {
	void dibr_merge(
		ftl::cuda::TextureObject<float4> &points,
		ftl::cuda::TextureObject<float4> &normals,
		ftl::cuda::TextureObject<int> &depth,
		ftl::render::SplatParams params,
		bool culling,
		cudaStream_t stream);

	template <typename T>
	void splat(
        ftl::cuda::TextureObject<float4> &normals,
		ftl::cuda::TextureObject<float> &density,
		ftl::cuda::TextureObject<T> &colour_in,
        ftl::cuda::TextureObject<int> &depth_in,        // Virtual depth map
        ftl::cuda::TextureObject<float> &depth_out,
		ftl::cuda::TextureObject<T> &colour_out,
        const ftl::render::SplatParams &params, cudaStream_t stream);

	template <typename A, typename B>
	void dibr_attribute(
		ftl::cuda::TextureObject<A> &in,	// Original colour image
		ftl::cuda::TextureObject<float4> &points,		// Original 3D points
		ftl::cuda::TextureObject<int> &depth_in,		// Virtual depth map
		ftl::cuda::TextureObject<B> &out,	// Accumulated output
		ftl::cuda::TextureObject<float> &contrib,
		ftl::render::SplatParams &params, cudaStream_t stream);

	template <typename A, typename B>
	void dibr_normalise(
		ftl::cuda::TextureObject<A> &in,
		ftl::cuda::TextureObject<B> &out,
		ftl::cuda::TextureObject<float> &contribs,
		cudaStream_t stream);
}
}

#endif  // _FTL_RECONSTRUCTION_SPLAT_CUDA_HPP_
