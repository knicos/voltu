#ifndef _FTL_CUDA_MASK_HPP_
#define _FTL_CUDA_MASK_HPP_

#include <ftl/cuda_common.hpp>
#include <ftl/rgbd/camera.hpp>

namespace ftl {
namespace cuda {

/**
 * Wrap an int mask value used to flag individual depth pixels. This acts like
 * a stencil buffer.
 */
class Mask {
	public:
	__device__ inline Mask() : v_(0u) {}
	__device__ explicit inline Mask(uint8_t v) : v_(v) {}
	#ifdef __CUDACC__
	__device__ inline Mask(const ftl::cuda::TextureObject<uint8_t> &m, int x, int y) : v_(m.tex2D(x,y)) {}
	#endif
	__device__ inline operator int() const { return v_; }

    __device__ inline bool is(uint8_t m) const { return v_ & m; }

	__device__ inline bool isFilled() const { return v_ & kMask_Filled; }
	__device__ inline bool isDiscontinuity() const { return v_ & kMask_Discontinuity; }
	__device__ inline bool hasCorrespondence() const { return v_ & kMask_Correspondence; }
	__device__ inline bool isBad() const { return v_ & kMask_Bad; }
	__device__ inline bool isNoise() const { return v_ & kMask_Noise; }

	__device__ inline void isFilled(bool v) { v_ = (v) ? v_ | kMask_Filled : v_ & (~kMask_Filled); }
	__device__ inline void isDiscontinuity(bool v) { v_ = (v) ? v_ | kMask_Discontinuity : v_ & (~kMask_Discontinuity); }
	__device__ inline void hasCorrespondence(bool v) { v_ = (v) ? v_ | kMask_Correspondence : v_ & (~kMask_Correspondence); }
	__device__ inline void isBad(bool v) { v_ = (v) ? v_ | kMask_Bad : v_ & (~kMask_Bad); }
	__device__ inline void isNoise(bool v) { v_ = (v) ? v_ | kMask_Noise : v_ & (~kMask_Noise); }

    static constexpr uint8_t kMask_Filled = 0x01;
	static constexpr uint8_t kMask_Discontinuity = 0x02;
	static constexpr uint8_t kMask_Correspondence = 0x04;
	static constexpr uint8_t kMask_Bad = 0x08;
	static constexpr uint8_t kMask_Noise = 0x10;

	private:
	uint8_t v_;
};

void discontinuity(
		ftl::cuda::TextureObject<uint8_t> &mask,
		ftl::cuda::TextureObject<uchar4> &support,
		ftl::cuda::TextureObject<float> &depth,
		const cv::Size size,
		const double minDepth,
		const double maxDepth,
		int radius, float depthCoef,
		cudaStream_t stream);

void discontinuity(
		ftl::cuda::TextureObject<uint8_t> &mask,
		ftl::cuda::TextureObject<uchar4> &support,
		ftl::cuda::TextureObject<float> &depth,
		const cv::Size size,
		const double minDepth,
		const double maxDepth,
		float depthCoef,
		float discon_thresh,
		float noise_thresh,
		float area_max,
		cudaStream_t stream);

void cull_mask(
		ftl::cuda::TextureObject<uint8_t> &mask,
		ftl::cuda::TextureObject<float> &depth,
		uint8_t id,
		unsigned int radius,
		cudaStream_t stream);

}
}

#endif
