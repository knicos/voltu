#ifndef _FTL_RENDER_PARAMS_HPP_
#define _FTL_RENDER_PARAMS_HPP_

#include <ftl/cuda_util.hpp>
#include <ftl/cuda_matrix_util.hpp>
#include <ftl/rgbd/camera.hpp>

namespace ftl {
namespace render {

static const uint kShowDisconMask = 0x00000001;
static const uint kNormalWeightColours = 0x00000002;

struct ViewPort {
	short x;
	short y;
	short width;
	short height;

	__device__ __host__ inline short x2() const { return x+width-1; }
	__device__ __host__ inline short y2() const { return y+height-1; }

	__device__ __host__ inline bool inside(short px, short py) const {
		return px >= x && px <= x2() && py >= y && py <= y2();
	}

	__device__ __host__ inline uint2 map(const ftl::rgbd::Camera &cam, const float2 &pt) const {
		return make_uint2(
			(pt.x - static_cast<float>(x)) * (static_cast<float>(cam.width) / static_cast<float>(width)),
			(pt.y - static_cast<float>(y)) * (static_cast<float>(cam.height) / static_cast<float>(height))
		);
	}

	__device__ __host__ inline uint2 reverseMap(const ftl::rgbd::Camera &cam, const float2 &pt) const {
		return make_uint2(
			(pt.x * (static_cast<float>(width) / static_cast<float>(cam.width))) + static_cast<float>(x),
			(pt.y * (static_cast<float>(height) / static_cast<float>(cam.height))) + static_cast<float>(y)
		);
	}
};

/**
 * Control how the viewport should be used.
 */
enum class ViewPortMode : uint8_t {
	Disabled = 0,		// Do not use the viewport data
	Clipping = 1,		// Clip the rendering to within the viewport for stencil like effect
	Warping = 2			// Stretch viewport region over entire frame
};

struct Parameters {
	uint m_flags;
	float disconDisparities;
	float depthCoef;
	int triangle_limit;

	ftl::rgbd::Camera camera;  // Virtual camera intrinsics
	ftl::render::ViewPort viewport;
	ftl::render::ViewPortMode viewPortMode;
};

}
}

#endif