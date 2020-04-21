#ifndef _FTL_LIBSTEREO_COSTS_CENSUS_HPP_
#define _FTL_LIBSTEREO_COSTS_CENSUS_HPP_

#include <opencv2/core/core.hpp>
#include "array2d.hpp"
#include "dsbase.hpp"

#include <cuda_runtime.h>

namespace impl {
	struct CensusMatchingCost : DSImplBase<unsigned short> {
		typedef unsigned short Type;

		CensusMatchingCost(ushort w, ushort h, ushort dmin, ushort dmax) : DSImplBase<unsigned short>({w,h,dmin,dmax}) {}

		__host__ __device__ inline unsigned short operator()(const int y, const int x, const int d) const {
			if ((x-d) < 0) { return 255; } // TODO: use window size
			const uint64_t bits =
				ct_l(y, x) ^ ct_r(y, x-d);
			#if defined(__CUDA_ARCH__)
				return __popcll(bits);
			#elif defined(_MSC_VER)
				return __popcnt64(bits);
			#elif defined(__GNUC__)
				return __builtin_popcountl(bits);
			#else
				static_assert(false, "unsupported compiler (popcount intrinsic)");
			#endif
		}

		static constexpr unsigned short COST_MAX = 64;

		Array2D<uint64_t>::Data ct_l;
		Array2D<uint64_t>::Data ct_r;
	};
}

class CensusMatchingCost : public DSBase<impl::CensusMatchingCost> {
public:
	typedef impl::CensusMatchingCost DataType;
	typedef unsigned short Type;

	CensusMatchingCost() : DSBase<DataType>(0, 0, 0, 0) {};
	CensusMatchingCost(int width, int height, int disp_min, int disp_max)
		: DSBase<DataType>(width, height, disp_min, disp_max),
			ct_l_(width, height), ct_r_(width,height),
			//cost_max(wwidth * wheight),
			ww_(9), wh_(7)
		{
			if (ww_ != 9 || wh_ != 7) {
				// TODO: window size paramters (hardcoded) in matching_cost.cpp
				throw std::exception();
			}
			if (ww_*wh_ > 64) {
				throw std::exception(); // todo: dynamic size
			}

			data().ct_l = ct_l_.data();
			data().ct_r = ct_r_.data();
		}

	void set(cv::InputArray l, cv::InputArray r);
	void set(const Array2D<uchar>& l, const Array2D<uchar>& r);
	static constexpr short COST_MAX = 9*7; // TODO: window size paramters (hardcoded) in matching_cost.cpp

protected:
	Array2D<uint64_t> ct_l_;
	Array2D<uint64_t> ct_r_;
	int ww_;
	int wh_;
};

#endif
