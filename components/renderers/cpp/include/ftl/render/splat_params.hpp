#ifndef _FTL_RENDER_SPLAT_PARAMS_HPP_
#define _FTL_RENDER_SPLAT_PARAMS_HPP_

#include <ftl/cuda_util.hpp>
#include <ftl/cuda_matrix_util.hpp>
#include <ftl/rgbd/camera.hpp>

namespace ftl {
namespace render {

static const uint kShowBlockBorders = 0x00000001;  // Deprecated: from voxels system
static const uint kNoSplatting = 0x00000002;
static const uint kNoUpsampling = 0x00000004;
static const uint kNoTexturing = 0x00000008;

struct __align__(16) SplatParams {
	float4x4 m_viewMatrix;
	float4x4 m_viewMatrixInverse;

	uint m_flags;
	//float voxelSize;
	float depthThreshold;

	ftl::rgbd::Camera camera;
};

}
}

#endif
