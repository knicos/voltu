#ifndef _FTL_CUDA_SMOOTHING_HPP_
#define _FTL_CUDA_SMOOTHING_HPP_

#include <ftl/rgbd/camera.hpp>
#include <ftl/cuda_common.hpp>

namespace ftl {
namespace cuda {

void colour_mls_smooth(
		ftl::cuda::TextureObject<half4> &normals_in,
		ftl::cuda::TextureObject<half4> &normals_out,
		ftl::cuda::TextureObject<float> &depth_in,
		ftl::cuda::TextureObject<float> &depth_out,
		ftl::cuda::TextureObject<uchar4> &colour_in,
		float smoothing,
		float colour_smoothing,
		int radius,
		const ftl::rgbd::Camera &camera,
		cudaStream_t stream);

void colour_mls_smooth_csr(
		ftl::cuda::TextureObject<uchar4> &region,
		ftl::cuda::TextureObject<half4> &normals_in,
		ftl::cuda::TextureObject<half4> &normals_out,
		ftl::cuda::TextureObject<float> &depth_in,
		ftl::cuda::TextureObject<float> &depth_out,
		//ftl::cuda::TextureObject<uchar4> &colour_in,
		const cv::cuda::GpuMat &colour_in,
		float smoothing,
		float colour_smoothing,
		bool filling,
		const ftl::rgbd::Camera &camera,
		cudaStream_t stream);

void mls_adjust_depth(
		ftl::cuda::TextureObject<half4> &normals_in,
		ftl::cuda::TextureObject<float4> &centroid_in,
		ftl::cuda::TextureObject<float> &depth_out,
		ftl::cuda::TextureObject<float> &depth_in,
		const ftl::rgbd::Camera &camera,
		cudaStream_t stream);

void mls_aggr_horiz(
		ftl::cuda::TextureObject<uchar4> &region,
		ftl::cuda::TextureObject<half4> &normals_in,
		ftl::cuda::TextureObject<half4> &normals_out,
		ftl::cuda::TextureObject<float> &depth_in,
		ftl::cuda::TextureObject<float4> &centroid_out,
		//ftl::cuda::TextureObject<uchar4> &colour_in,
		const cv::cuda::GpuMat &colour_in,
		float smoothing,
		float colour_smoothing,
		int radius,
		const ftl::rgbd::Camera &camera,
		cudaStream_t stream);

void mls_aggr_vert(
		ftl::cuda::TextureObject<uchar4> &region,
		ftl::cuda::TextureObject<half4> &normals_in,
		ftl::cuda::TextureObject<half4> &normals_out,
		ftl::cuda::TextureObject<float4> &centroid_in,
		ftl::cuda::TextureObject<float4> &centroid_out,
		float smoothing,
		float colour_smoothing,
		int radius,
		const ftl::rgbd::Camera &camera,
		cudaStream_t stream);

void adaptive_mls_smooth(
		ftl::cuda::TextureObject<half4> &normals_in,
		ftl::cuda::TextureObject<half4> &normals_out,
		ftl::cuda::TextureObject<float> &depth_in,
		ftl::cuda::TextureObject<float> &depth_out,
		ftl::cuda::TextureObject<float> &smoothing,
		int radius,
		const ftl::rgbd::Camera &camera,
		cudaStream_t stream);

void smooth_channel(
		ftl::cuda::TextureObject<uchar4> &colour_in,
		//ftl::cuda::TextureObject<float> &depth_in,
		ftl::cuda::TextureObject<float> &smoothing_out,
		const ftl::rgbd::Camera &camera,
		float alpha,
		float scale,
		int radius,
		cudaStream_t stream);

void depth_smooth(
	ftl::cuda::TextureObject<float> &depth_in,
	ftl::cuda::TextureObject<uchar4> &colour_in,
	ftl::cuda::TextureObject<float> &depth_out,
	const ftl::rgbd::Camera &camera,
	int radius, float factor, float thresh, int iters,
	cudaStream_t stream);

void smoothing_factor(
	ftl::cuda::TextureObject<float> &depth_in,
	//ftl::cuda::TextureObject<float> &depth_tmp,
	ftl::cuda::TextureObject<float> &temp,
	//ftl::cuda::TextureObject<uchar4> &colour_in,
	ftl::cuda::TextureObject<float> &smoothing,
	float thresh,
	const ftl::rgbd::Camera &camera,
	cudaStream_t stream);

}
}

#endif  // _FTL_CUDA_SMOOTHING_HPP_
