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
	float sub_pixel;
	float P1;
	float P2;
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

void correspondence(
        ftl::cuda::TextureObject<uchar4> &c1,
        ftl::cuda::TextureObject<uchar4> &c2,
        ftl::cuda::TextureObject<float> &d1,
        ftl::cuda::TextureObject<float> &d2,
        ftl::cuda::TextureObject<short2> &screen,
		ftl::cuda::TextureObject<float> &conf,
		//ftl::cuda::TextureObject<uint8_t> &mask,
        //ftl::cuda::TextureObject<half> &costs,
        float4x4 &pose,
        const ftl::rgbd::Camera &cam1,
        const ftl::rgbd::Camera &cam2, const ftl::cuda::MvMLSParams &params, int radius,
        cudaStream_t stream);

void correspondence(
        ftl::cuda::TextureObject<float> &d1,
        ftl::cuda::TextureObject<float> &d2,
        ftl::cuda::TextureObject<short2> &screen,
		ftl::cuda::TextureObject<float> &conf,
		ftl::cuda::TextureObject<uint8_t> &mask,
        float4x4 &pose,
        const ftl::rgbd::Camera &cam1,
        const ftl::rgbd::Camera &cam2, const ftl::cuda::MvMLSParams &params, int func,
        cudaStream_t stream);

void aggregate_colour_costs(
        ftl::cuda::TextureObject<float> &d1,
        ftl::cuda::TextureObject<float> &d2,
        ftl::cuda::TextureObject<half> &costs1,
        ftl::cuda::TextureObject<half> &costs2,
        ftl::cuda::TextureObject<short2> &screen,
		ftl::cuda::TextureObject<float> &conf,
        float4x4 &pose,
        const ftl::rgbd::Camera &cam1,
        const ftl::rgbd::Camera &cam2, const ftl::cuda::MvMLSParams &params, int radius,
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
		ftl::cuda::TextureObject<half4> &n1,
		ftl::cuda::TextureObject<half4> &n2,
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
        ftl::cuda::TextureObject<half4> &normals1,
        ftl::cuda::TextureObject<half4> &normals2,
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
    ftl::cuda::TextureObject<half4> &norms,
    ftl::cuda::TextureObject<float4> &cents,
    ftl::cuda::TextureObject<float> &contribs,
    cudaStream_t stream);

void show_cor_error(
	ftl::cuda::TextureObject<uchar4> &colour,
	ftl::cuda::TextureObject<short2> &screen1,
	ftl::cuda::TextureObject<short2> &screen2,
	float thresh,
	cudaStream_t stream);

void remove_cor_error(
	ftl::cuda::TextureObject<float> &adjust,
	ftl::cuda::TextureObject<short2> &screen1,
	ftl::cuda::TextureObject<short2> &screen2,
	float thresh,
	cudaStream_t stream);

void show_depth_adjustment(
	ftl::cuda::TextureObject<uchar4> &colour,
    ftl::cuda::TextureObject<short2> &screen,
	ftl::cuda::TextureObject<float> &adjust,
	float scale,
	cudaStream_t stream);

void viz_reprojection(
	ftl::cuda::TextureObject<uchar4> &colour_out,
	ftl::cuda::TextureObject<uchar4> &colour_in,
	ftl::cuda::TextureObject<float> &depth_out,
	ftl::cuda::TextureObject<float> &depth_in,
	const float4x4 &pose,
	const ftl::rgbd::Camera &cam1,
	const ftl::rgbd::Camera &cam2, cudaStream_t stream);

}
}

#endif
