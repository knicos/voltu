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
	typedef uint8_t type;

	__device__ inline Mask() : v_(0u) {}
	__device__ explicit inline Mask(type v) : v_(v) {}
	#ifdef __CUDACC__
	__device__ inline Mask(const ftl::cuda::TextureObject<type> &m, int x, int y) : v_(m.tex2D(x,y)) {}
	#endif
	__device__ inline operator int() const { return v_; }

    __device__ inline bool is(type m) const { return v_ & m; }

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

    static constexpr type kMask_Filled = 0x01;
	static constexpr type kMask_Discontinuity = 0x02;
	static constexpr type kMask_Correspondence = 0x04;
	static constexpr type kMask_Bad = 0x08;
	static constexpr type kMask_Noise = 0x10;
	static constexpr type kMask_Occlusion = 0x20;

	private:
	type v_;
};

void discontinuity(
		ftl::cuda::TextureObject<ftl::cuda::Mask::type> &mask,
		ftl::cuda::TextureObject<uchar4> &support,
		ftl::cuda::TextureObject<float> &depth,
		const cv::Size size,
		const double minDepth,
		const double maxDepth,
		int radius, float depthCoef,
		cudaStream_t stream);

void discontinuity(
		ftl::cuda::TextureObject<ftl::cuda::Mask::type> &mask,
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

void border_mask(
		ftl::cuda::TextureObject<ftl::cuda::Mask::type> &mask,
		int left, int right, int top, int bottom,
		cudaStream_t stream);

void cull_mask(
		ftl::cuda::TextureObject<ftl::cuda::Mask::type> &mask,
		ftl::cuda::TextureObject<float> &depth,
		ftl::cuda::Mask::type id,
		bool invert,
		unsigned int radius,
		cudaStream_t stream);

void show_mask(
        ftl::cuda::TextureObject<uchar4> &colour,
		ftl::cuda::TextureObject<uint8_t> &mask,
        int id, uchar4 style, cudaStream_t stream);

}
}

#endif
