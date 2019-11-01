#ifndef _FTL_RENDER_SPLAT_PARAMS_HPP_
#define _FTL_RENDER_SPLAT_PARAMS_HPP_

#include <ftl/cuda_util.hpp>
#include <ftl/cuda_matrix_util.hpp>
#include <ftl/rgbd/camera.hpp>

namespace ftl {
namespace render {

static const uint kShowDisconMask = 0x00000001;
static const uint kNormalWeightColours = 0x00000002;

struct __align__(16) SplatParams {
	float4x4 m_viewMatrix;
	float4x4 m_viewMatrixInverse;

	uint m_flags;
	//float voxelSize;
	float depthThreshold;
	int triangle_limit;

	ftl::rgbd::Camera camera;
};

}
}

#endif
