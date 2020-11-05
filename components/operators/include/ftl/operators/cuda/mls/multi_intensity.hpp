#ifndef _FTL_CUDA_MLS_MULTIINTENSITY_HPP_
#define _FTL_CUDA_MLS_MULTIINTENSITY_HPP_

#include <ftl/cuda_common.hpp>
#include <ftl/rgbd/camera.hpp>
#include <ftl/cuda_matrix_util.hpp>

namespace ftl {
namespace cuda {

// TODO: 4th channel of normals could be used as curvature estimate? Then can
// adaptively adjust the smoothing amount by the degree of curvature. This
// curvature value must be low for noise data, is only high if a consistent
// high curvature is detected over a radius. Other option is to also or instead
// use the curvature value as a feature, to only smooth against points with
// similar curvature?

/**
 * For a particular viewpoint, use a set of other viewpoints to estimate a
 * smooth surface for the focus viewpoint. This version of MLS uses absolute
 * difference of some 8-bit value as a weight, this can be raw intensity or
 * a local contrast measure. This `prime`, `gather`, `adjust` cycle should
 * be iterated 2-3 times, but perhaps better to do one iteration of each view
 * first before repeating a view.
 * 
 * Note: Have one instance per frameset because the internal buffers are
 * allocated based upon frame image size.
 */
class MLSMultiIntensity
{
public:
	MLSMultiIntensity(int radius);
	~MLSMultiIntensity();

	void prime(
		const cv::cuda::GpuMat &depth_prime,
		const cv::cuda::GpuMat &intensity_prime,
		const ftl::rgbd::Camera &cam_prime,
		const float4x4 &pose_prime,
		cudaStream_t stream
	);

	void gatherPrime(float smoothing, cudaStream_t stream);

	void gather(
		const cv::cuda::GpuMat &depth_src,
		const cv::cuda::GpuMat &normals_src,
		const cv::cuda::GpuMat &intensity_src,
		const ftl::rgbd::Camera &cam_src,
		const float4x4 &pose_src,
		float smoothing,
		float fsmoothing,
		cudaStream_t stream
	);

	void adjust(
		cv::cuda::GpuMat &depth_out,
		cv::cuda::GpuMat &normals_out,
		cudaStream_t stream
	);

private:
	cv::cuda::GpuMat depth_prime_;
	cv::cuda::GpuMat intensity_prime_;
	ftl::rgbd::Camera cam_prime_;
	float4x4 pose_prime_;
	int radius_;

	cv::cuda::GpuMat normal_accum_;
	cv::cuda::GpuMat centroid_accum_;
	cv::cuda::GpuMat weight_accum_;
};

void mean_subtract(
	const cv::cuda::GpuMat &intensity,
	cv::cuda::GpuMat &contrast,
	int radius,
	cudaStream_t stream
);

}
}

#endif