#ifndef _FTL_RECONSTRUCTION_SPLAT_CUDA_HPP_
#define _FTL_RECONSTRUCTION_SPLAT_CUDA_HPP_

#include <ftl/depth_camera.hpp>
#include <ftl/voxel_hash.hpp>
//#include <ftl/ray_cast_util.hpp>

#include "splat_params.hpp"

namespace ftl {
namespace cuda {

/**
 * NOTE: Not strictly isosurface currently since it includes the internals
 * of objects up to at most truncation depth.
 */
void isosurface_point_image(const ftl::voxhash::HashData& hashData,
			const ftl::cuda::TextureObject<uint> &depth,
			const ftl::render::SplatParams &params, cudaStream_t stream);

//void isosurface_point_image_stereo(const ftl::voxhash::HashData& hashData,
//		const ftl::voxhash::HashParams& hashParams,
//		const RayCastData &rayCastData, const RayCastParams &params,
//		cudaStream_t stream);

// TODO: isosurface_point_cloud

void splat_points(const ftl::cuda::TextureObject<uint> &depth_in,
		const ftl::cuda::TextureObject<float> &depth_out,
		const ftl::render::SplatParams &params, cudaStream_t stream);

void dibr(const ftl::cuda::TextureObject<float> &depth_in,
		const ftl::cuda::TextureObject<uchar4> &colour_out, int numcams,
		const ftl::render::SplatParams &params, cudaStream_t stream);

}
}

#endif  // _FTL_RECONSTRUCTION_SPLAT_CUDA_HPP_
