#ifndef _FTL_RECONSTRUCTION_INTEGRATORS_HPP_
#define _FTL_RECONSTRUCTION_INTEGRATORS_HPP_

#include <ftl/voxel_hash.hpp>
#include <ftl/depth_camera.hpp>

namespace ftl {
namespace cuda {

void integrateDepthMap(ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams,
		const DepthCameraData& depthCameraData, const DepthCameraParams& depthCameraParams, cudaStream_t stream);

}
}

#endif  // _FTL_RECONSTRUCTION_INTEGRATORS_HPP_
