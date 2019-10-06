#ifndef _FTL_ILW_CUDA_HPP_
#define _FTL_ILW_CUDA_HPP_

#include <ftl/cuda_common.hpp>
#include <ftl/rgbd/camera.hpp>
#include <ftl/cuda_matrix_util.hpp>

namespace ftl {
namespace cuda {

struct ILWParams {
    float spatial_smooth;
    float colour_smooth;
    float cost_ratio;
    float threshold;
	float range;
    uint flags;
};

static const uint kILWFlag_IgnoreBad = 0x0001;
static const uint kILWFlag_RestrictZ = 0x0002;
static const uint kILWFlag_SkipBadColour = 0x0004;
static const uint kILWFlag_ColourConfidenceOnly = 0x0008;

/**
 * Wrap an int mask value used to flag individual depth pixels.
 */
class ILWMask {
	public:
	__device__ explicit inline ILWMask(int v) : v_(v) {}
	#ifdef __CUDACC__
	__device__ inline ILWMask(const ftl::cuda::TextureObject<int> &m, int x, int y) : v_(m.tex2D(x,y)) {}
	#endif
	__device__ inline operator int() const { return v_; }

	__device__ inline bool isFilled() const { return v_ & kMask_Filled; }
	__device__ inline bool isDiscontinuity() const { return v_ & kMask_Discontinuity; }
	__device__ inline bool hasCorrespondence() const { return v_ & kMask_Correspondence; }
	__device__ inline bool isBad() const { return v_ & kMask_Bad; }

	__device__ inline void isFilled(bool v) { v_ = (v) ? v_ | kMask_Filled : v_ & (~kMask_Filled); }
	__device__ inline void isDiscontinuity(bool v) { v_ = (v) ? v_ | kMask_Discontinuity : v_ & (~kMask_Discontinuity); }
	__device__ inline void hasCorrespondence(bool v) { v_ = (v) ? v_ | kMask_Correspondence : v_ & (~kMask_Correspondence); }
	__device__ inline void isBad(bool v) { v_ = (v) ? v_ | kMask_Bad : v_ & (~kMask_Bad); }

	private:
	int v_;

	static const int kMask_Filled = 0x0001;
	static const int kMask_Discontinuity = 0x0002;
	static const int kMask_Correspondence = 0x0004;
	static const int kMask_Bad = 0x0008;
};

void correspondence(
    ftl::cuda::TextureObject<float> &d1,
    ftl::cuda::TextureObject<float> &d2,
    ftl::cuda::TextureObject<uchar4> &c1,
    ftl::cuda::TextureObject<uchar4> &c2,
    ftl::cuda::TextureObject<float> &dout,
    ftl::cuda::TextureObject<float> &conf,
    float4x4 &pose1,
    float4x4 &pose1_inv,
    float4x4 &pose2,
    const ftl::rgbd::Camera &cam1,
    const ftl::rgbd::Camera &cam2,
    const ILWParams &params, int win,
    cudaStream_t stream
);

void move_points(
    ftl::cuda::TextureObject<float> &d_old,
    ftl::cuda::TextureObject<float> &d_new,
	ftl::cuda::TextureObject<float> &conf,
    const ftl::rgbd::Camera &camera,
    const float4x4 &pose,
	const ILWParams &params,
    float rate,
    int radius,
    cudaStream_t stream
);

}
}

#endif  // _FTL_ILW_CUDA_HPP_
