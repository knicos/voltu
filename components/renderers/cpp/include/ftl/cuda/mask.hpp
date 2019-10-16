#ifndef _FTL_CUDA_MASK_HPP_
#define _FTL_CUDA_MASK_HPP_

namespace ftl {
namespace cuda {

/**
 * Wrap an int mask value used to flag individual depth pixels.
 */
class Mask {
	public:
	__device__ inline Mask() : v_(0) {}
	__device__ explicit inline Mask(int v) : v_(v) {}
	#ifdef __CUDACC__
	__device__ inline Mask(const ftl::cuda::TextureObject<int> &m, int x, int y) : v_(m.tex2D(x,y)) {}
	#endif
	__device__ inline operator int() const { return v_; }

    __device__ inline bool is(int m) const { return v_ & m; }

	__device__ inline bool isFilled() const { return v_ & kMask_Filled; }
	__device__ inline bool isDiscontinuity() const { return v_ & kMask_Discontinuity; }
	__device__ inline bool hasCorrespondence() const { return v_ & kMask_Correspondence; }
	__device__ inline bool isBad() const { return v_ & kMask_Bad; }

	__device__ inline void isFilled(bool v) { v_ = (v) ? v_ | kMask_Filled : v_ & (~kMask_Filled); }
	__device__ inline void isDiscontinuity(bool v) { v_ = (v) ? v_ | kMask_Discontinuity : v_ & (~kMask_Discontinuity); }
	__device__ inline void hasCorrespondence(bool v) { v_ = (v) ? v_ | kMask_Correspondence : v_ & (~kMask_Correspondence); }
	__device__ inline void isBad(bool v) { v_ = (v) ? v_ | kMask_Bad : v_ & (~kMask_Bad); }

    static constexpr int kMask_Filled = 0x0001;
	static constexpr int kMask_Discontinuity = 0x0002;
	static constexpr int kMask_Correspondence = 0x0004;
	static constexpr int kMask_Bad = 0x0008;

	private:
	int v_;
};

}
}

#endif
