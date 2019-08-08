#ifndef _FTL_RENDER_SPLAT_PARAMS_HPP_
#define _FTL_RENDER_SPLAT_PARAMS_HPP_

#include <ftl/cuda_util.hpp>
#include <ftl/cuda_matrix_util.hpp>
#include <ftl/depth_camera_params.hpp>

namespace ftl {
namespace render {

static const uint kShowBlockBorders = 0x0001;
static const uint kNoSplatting = 0x0002;

struct __align__(16) SplatParams {
	float4x4 m_viewMatrix;
	float4x4 m_viewMatrixInverse;

	uint m_flags;
	float voxelSize;
	float depthThreshold;

	DepthCameraParams camera;
};

}
}

#endif
