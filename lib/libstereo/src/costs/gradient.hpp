#ifndef _FTL_LIBSTEREO_COSTS_GRADIENT_HPP_
#define _FTL_LIBSTEREO_COSTS_GRADIENT_HPP_

#include <opencv2/core/core.hpp>
#include "array2d.hpp"
#include "dsbase.hpp"

#include <cuda_runtime.h>

namespace impl {
	// Basic data class for use on GPU and CPU
	struct GradientMatchingCostL2 : DSImplBase<unsigned short> {
		typedef unsigned short Type;

		GradientMatchingCostL2(ushort w, ushort h, ushort dmin, ushort dmax) : DSImplBase<unsigned short>({w,h,dmin,dmax}) {}

		__host__ __device__ inline unsigned short operator()(const int y, const int x, const int d) const {
			if ((x-d) < 0) { return 255; }
			const short a = l_dx(y,x) - r_dx(y,x-d);
			const short b = l_dy(y,x) - r_dy(y,x-d);
			#ifdef USE_GPU
			return sqrtf(a*a + b*b);
			#else
			return sqrtf(a*a + b*b);
			#endif
		}

		Array2D<short>::Data l_dx;
		Array2D<short>::Data r_dx;
		Array2D<short>::Data l_dy;
		Array2D<short>::Data r_dy;
	};
}

class GradientMatchingCostL2 : public DSBase<impl::GradientMatchingCostL2> {
public:
	typedef impl::GradientMatchingCostL2 DataType;
	typedef unsigned short Type;

	GradientMatchingCostL2() : DSBase<DataType>(0, 0, 0, 0) {};
	GradientMatchingCostL2(int width, int height, int disp_min, int disp_max)
		: DSBase<DataType>(width, height, disp_min, disp_max),
			l_dx_(width, height), l_dy_(width, height), r_dx_(width, height),
			r_dy_(width, height)
	{
		// Copy the pointers etc to the data object
		data().l_dx = l_dx_.data();
		data().l_dy = l_dy_.data();
		data().r_dx = r_dx_.data();
		data().r_dy = r_dy_.data();
	};

	void set(const Array2D<uchar>& l, const Array2D<uchar>& r);

	// Maximum value calculated from Scharr kernel weights
	static constexpr short COST_MAX = 255*3 + 255*10 + 255*3;

protected:
	Array2D<short> l_dx_;
	Array2D<short> r_dx_;
	Array2D<short> l_dy_;
	Array2D<short> r_dy_;
};

#endif