#ifndef _FTL_RECONSTRUCTION_SPLAT_CUDA_HPP_
#define _FTL_RECONSTRUCTION_SPLAT_CUDA_HPP_

#include <ftl/cuda_common.hpp>
#include <ftl/render/render_params.hpp>

namespace ftl {
namespace cuda {
	void screen_coord(
		ftl::cuda::TextureObject<float> &depth,
		ftl::cuda::TextureObject<float> &depth_out,
		ftl::cuda::TextureObject<short2> &screen_out,
		const ftl::render::Parameters &params,
		const float4x4 &pose,
		const ftl::rgbd::Camera &camera,
		cudaStream_t stream);

	void screen_coord(
		ftl::cuda::TextureObject<float> &depth_out,
		ftl::cuda::TextureObject<short2> &screen_out,
		const ftl::render::Parameters &params,
		const float4x4 &pose,
		const ftl::rgbd::Camera &camera,
		cudaStream_t stream);

	void triangle_render1(
		ftl::cuda::TextureObject<float> &depth_in,
		ftl::cuda::TextureObject<int> &depth_out,
		ftl::cuda::TextureObject<short2> &screen,
		const ftl::render::Parameters &params,
		cudaStream_t stream);

	void mesh_blender(
		ftl::cuda::TextureObject<int> &depth_in,
		ftl::cuda::TextureObject<int> &depth_out,
		const ftl::rgbd::Camera &camera,
		float alpha,
		cudaStream_t stream);

	void mesh_blender(
		ftl::cuda::TextureObject<int> &depth_in,
		ftl::cuda::TextureObject<float> &depth_out,
		ftl::cuda::TextureObject<short> &weights_in,
		ftl::cuda::TextureObject<float> &weights_out,
		const ftl::render::Parameters &params,
		const ftl::rgbd::Camera &camera,
		const float4x4 &transform,
		float alpha,
		cudaStream_t stream);
	
	/*void dibr_merge(
		ftl::cuda::TextureObject<float4> &points,
		ftl::cuda::TextureObject<float4> &normals,
		ftl::cuda::TextureObject<int> &depth,
		ftl::render::Parameters params,
		bool culling,
		cudaStream_t stream);

	void dibr_merge(
		ftl::cuda::TextureObject<float4> &points,
		ftl::cuda::TextureObject<int> &depth,
		ftl::render::Parameters params,
		cudaStream_t stream);*/

	void dibr_merge(
		ftl::cuda::TextureObject<float> &depth,
		ftl::cuda::TextureObject<int> &depth_out,
		const float4x4 &transform,
		const ftl::rgbd::Camera &cam,
		ftl::render::Parameters params,
		cudaStream_t stream);

	void dibr_merge(
		ftl::cuda::TextureObject<int> &depth_out,
		const float4x4 &transform,
		const ftl::rgbd::Camera &cam,
		ftl::render::Parameters params,
		cudaStream_t stream);

	/*template <typename T>
	void splat(
        ftl::cuda::TextureObject<half4> &normals,
		ftl::cuda::TextureObject<float> &density,
		ftl::cuda::TextureObject<T> &colour_in,
        ftl::cuda::TextureObject<int> &depth_in,        // Virtual depth map
        ftl::cuda::TextureObject<float> &depth_out,
		ftl::cuda::TextureObject<T> &colour_out,
        const ftl::render::Parameters &params, cudaStream_t stream);*/

	template <typename A, typename B>
	void dibr_attribute(
		ftl::cuda::TextureObject<A> &in,	// Original colour image
		ftl::cuda::TextureObject<float4> &points,		// Original 3D points
		ftl::cuda::TextureObject<int> &depth_in,		// Virtual depth map
		ftl::cuda::TextureObject<B> &out,	// Accumulated output
		ftl::cuda::TextureObject<float> &contrib,
		ftl::render::Parameters &params, cudaStream_t stream);

	template <typename A, typename B>
	void reproject(
		ftl::cuda::TextureObject<A> &in,	// Original colour image
		ftl::cuda::TextureObject<float> &depth_src,		// Original 3D points
		ftl::cuda::TextureObject<float> &depth_in,		// Virtual depth map
		ftl::cuda::TextureObject<short> &weights,
		ftl::cuda::TextureObject<half4> *normals,
		ftl::cuda::TextureObject<B> &out,	// Accumulated output
		ftl::cuda::TextureObject<int> &contrib,
		const ftl::render::Parameters &params,
		const ftl::rgbd::Camera &camera,
		const float4x4 &transform, const float3x3 &transformR,
		cudaStream_t stream);

	/*template <typename A, typename B>
	void reproject(
		ftl::cuda::TextureObject<A> &in,	// Original colour image
		ftl::cuda::TextureObject<float> &depth_src,		// Original 3D points
		ftl::cuda::TextureObject<float> &depth_in,		// Virtual depth map
		ftl::cuda::TextureObject<B> &out,	// Accumulated output
		ftl::cuda::TextureObject<int> &contrib,
		const ftl::render::Parameters &params,
		const ftl::rgbd::Camera &camera,
		const float4x4 &poseInv, cudaStream_t stream);*/

	template <typename A, typename B>
	void reproject(
		ftl::cuda::TextureObject<A> &in,	// Original colour image
		ftl::cuda::TextureObject<float> &depth_in,		// Virtual depth map
		ftl::cuda::TextureObject<B> &out,	// Accumulated output
		ftl::cuda::TextureObject<int> &contrib,
		const ftl::render::Parameters &params,
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
		ftl::cuda::TextureObject<int> &contribs,
		bool flipy,
		cudaStream_t stream);

	template <typename A, typename B>
	void dibr_normalise(
		ftl::cuda::TextureObject<A> &in,
		ftl::cuda::TextureObject<B> &out,
		ftl::cuda::TextureObject<float> &weights,
		cudaStream_t stream);

	void show_missing_colour(
		ftl::cuda::TextureObject<float> &depth,
		ftl::cuda::TextureObject<uchar4> &out,
		ftl::cuda::TextureObject<int> &contribs,
		uchar4 bad_colour,
		const ftl::rgbd::Camera &cam,
		cudaStream_t stream);

	void show_colour_weights(
		ftl::cuda::TextureObject<uchar4> &out,
		ftl::cuda::TextureObject<int> &contribs,
		uchar4 bad_colour,
		cudaStream_t stream);

	void merge_convert_depth(
        ftl::cuda::TextureObject<int> &d1,
		ftl::cuda::TextureObject<float> &d2,
        float factor, cudaStream_t stream);

	void fix_bad_colour(
		ftl::cuda::TextureObject<float> &depth,
		ftl::cuda::TextureObject<uchar4> &out,
		ftl::cuda::TextureObject<int> &contribs,
		uchar4 bad_colour,
		const ftl::rgbd::Camera &cam,
		cudaStream_t stream);
}
}

#endif  // _FTL_RECONSTRUCTION_SPLAT_CUDA_HPP_
