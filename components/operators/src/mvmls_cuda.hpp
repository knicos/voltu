#ifndef _FTL_CUDA_MVMLS_HPP_
#define _FTL_CUDA_MVMLS_HPP_

#include <ftl/rgbd/camera.hpp>
#include <ftl/cuda_common.hpp>
#include <ftl/cuda_matrix_util.hpp>

namespace ftl {
namespace cuda {

struct MvMLSParams {
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

void correspondence(
        ftl::cuda::TextureObject<float> &d1,
        ftl::cuda::TextureObject<float> &d2,
        ftl::cuda::TextureObject<uchar4> &c1,
        ftl::cuda::TextureObject<uchar4> &c2,
        ftl::cuda::TextureObject<short2> &screen,
		ftl::cuda::TextureObject<float> &conf,
		ftl::cuda::TextureObject<uint8_t> &mask,
        float4x4 &pose,
        const ftl::rgbd::Camera &cam1,
        const ftl::rgbd::Camera &cam2, const ftl::cuda::MvMLSParams &params, int func,
        cudaStream_t stream);

void zero_confidence(
		ftl::cuda::TextureObject<float> &conf,
		ftl::cuda::TextureObject<float> &depth,
		cudaStream_t stream);

/*void aggregate_sources(
    ftl::cuda::TextureObject<float4> &n1,
    ftl::cuda::TextureObject<float4> &n2,
    ftl::cuda::TextureObject<float4> &c1,
    ftl::cuda::TextureObject<float4> &c2,
    ftl::cuda::TextureObject<float> &contribs1,
    ftl::cuda::TextureObject<float> &contribs2,
    ftl::cuda::TextureObject<short2> &screen,
	//const float4x4 &pose1,
	//const float4x4 &poseInv2,
	const float4x4 &poseInv1,
	const float4x4 &pose2,
    cudaStream_t stream);*/

void aggregate_sources(
		ftl::cuda::TextureObject<float4> &n1,
		ftl::cuda::TextureObject<float4> &n2,
		ftl::cuda::TextureObject<float4> &c1,
		ftl::cuda::TextureObject<float4> &c2,
		ftl::cuda::TextureObject<float> &depth1,
		//ftl::cuda::TextureObject<float> &depth2,
		//ftl::cuda::TextureObject<short2> &screen,
		const float4x4 &transform,
		const ftl::rgbd::Camera &cam1,
		const ftl::rgbd::Camera &cam2,
		cudaStream_t stream);

void best_sources(
        ftl::cuda::TextureObject<float4> &normals1,
        ftl::cuda::TextureObject<float4> &normals2,
        ftl::cuda::TextureObject<uchar4> &support1,
        ftl::cuda::TextureObject<uchar4> &suppor2,
        ftl::cuda::TextureObject<float> &depth1,
        ftl::cuda::TextureObject<float> &depth2,
        ftl::cuda::TextureObject<short2> &screen,
        const float4x4 &transform,
        const ftl::rgbd::Camera &cam1,
        const ftl::rgbd::Camera &cam2,
        int id1,
        int id2,
        cudaStream_t stream);

void vis_best_sources(
        ftl::cuda::TextureObject<short2> &screen,
        ftl::cuda::TextureObject<uchar4> &colour,
        int myid,
        int count,
        cudaStream_t stream);

void normalise_aggregations(
    ftl::cuda::TextureObject<float4> &norms,
    ftl::cuda::TextureObject<float4> &cents,
    ftl::cuda::TextureObject<float> &contribs,
    cudaStream_t stream);

}
}

#endif
