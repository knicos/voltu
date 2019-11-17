#ifndef _FTL_RECONSTRUCTION_SPLAT_CUDA_HPP_
#define _FTL_RECONSTRUCTION_SPLAT_CUDA_HPP_

#include <ftl/cuda_common.hpp>
#include <ftl/render/splat_params.hpp>

namespace ftl {
namespace cuda {
	void screen_coord(
		ftl::cuda::TextureObject<float> &depth,
		ftl::cuda::TextureObject<float> &depth_out,
		ftl::cuda::TextureObject<short2> &screen_out,
		const ftl::render::SplatParams &params,
		const float4x4 &pose,
		const ftl::rgbd::Camera &camera,
		cudaStream_t stream);

	void triangle_render1(
		ftl::cuda::TextureObject<float> &depth_in,
		ftl::cuda::TextureObject<int> &depth_out,
		ftl::cuda::TextureObject<short2> &screen,
		const ftl::render::SplatParams &params,
		cudaStream_t stream);

	void mesh_blender(
		ftl::cuda::TextureObject<int> &depth_in,
		ftl::cuda::TextureObject<int> &depth_out,
		const ftl::rgbd::Camera &camera,
		float alpha,
		cudaStream_t stream);
	
	void dibr_merge(
		ftl::cuda::TextureObject<float4> &points,
		ftl::cuda::TextureObject<float4> &normals,
		ftl::cuda::TextureObject<int> &depth,
		ftl::render::SplatParams params,
		bool culling,
		cudaStream_t stream);

	void dibr_merge(
		ftl::cuda::TextureObject<float4> &points,
		ftl::cuda::TextureObject<int> &depth,
		ftl::render::SplatParams params,
		cudaStream_t stream);

	void dibr_merge(
		ftl::cuda::TextureObject<float> &depth,
		ftl::cuda::TextureObject<int> &depth_out,
		const float4x4 &transform,
		const ftl::rgbd::Camera &cam,
		ftl::render::SplatParams params,
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
	void reproject(
		ftl::cuda::TextureObject<A> &in,	// Original colour image
		ftl::cuda::TextureObject<float> &depth_src,		// Original 3D points
		ftl::cuda::TextureObject<float> &depth_in,		// Virtual depth map
		ftl::cuda::TextureObject<float4> &normals,
		ftl::cuda::TextureObject<B> &out,	// Accumulated output
		ftl::cuda::TextureObject<float> &contrib,
		const ftl::render::SplatParams &params,
		const ftl::rgbd::Camera &camera,
		const float4x4 &transform, const float3x3 &transformR, cudaStream_t stream);

	template <typename A, typename B>
	void reproject(
		ftl::cuda::TextureObject<A> &in,	// Original colour image
		ftl::cuda::TextureObject<float> &depth_src,		// Original 3D points
		ftl::cuda::TextureObject<float> &depth_in,		// Virtual depth map
		ftl::cuda::TextureObject<B> &out,	// Accumulated output
		ftl::cuda::TextureObject<float> &contrib,
		const ftl::render::SplatParams &params,
		const ftl::rgbd::Camera &camera,
		const float4x4 &poseInv, cudaStream_t stream);

	void equirectangular_reproject(
		ftl::cuda::TextureObject<uchar4> &image_in,
		ftl::cuda::TextureObject<uchar4> &image_out,
		const ftl::rgbd::Camera &camera, const float3x3 &pose, cudaStream_t stream);

	template <typename A, typename B>
	void dibr_normalise(
		ftl::cuda::TextureObject<A> &in,
		ftl::cuda::TextureObject<B> &out,
		ftl::cuda::TextureObject<float> &contribs,
		cudaStream_t stream);

	void show_missing_colour(
		ftl::cuda::TextureObject<float> &depth,
		ftl::cuda::TextureObject<uchar4> &out,
		ftl::cuda::TextureObject<float> &contribs,
		uchar4 bad_colour,
		const ftl::rgbd::Camera &cam,
		cudaStream_t stream);

	void show_mask(
        ftl::cuda::TextureObject<uchar4> &colour,
		ftl::cuda::TextureObject<int> &mask,
        int id, uchar4 style, cudaStream_t stream);
}
}

#endif  // _FTL_RECONSTRUCTION_SPLAT_CUDA_HPP_
