#ifndef _FTL_LIBSTEREO_COSTS_ADBT_HPP_
#define _FTL_LIBSTEREO_COSTS_ADBT_HPP_

#include <opencv2/core/core.hpp>
#include "array2d.hpp"
#include "dsbase.hpp"

#include <cuda_runtime.h>

namespace impl {
	// Basic data class for use on GPU and CPU
	struct AbsDiffBT : DSImplBase<unsigned short> {
		typedef unsigned short Type;

		AbsDiffBT(ushort w, ushort h, ushort dmin, ushort dmax) : DSImplBase<unsigned short>({w,h,dmin,dmax}) {}

		__host__ __device__ inline unsigned short operator()(const int y, const int x, const int d) const {
			if ((x-d) < 0) { return 255; }
			int l_val = l(y,x);
			int l_val_p = (x > 0) ? l(y,x-1) : l_val;
			int l_val_n = (x < width-1) ? l(y,x+1) : l_val;
			int r_val = r(y,x-d);

			// Pseudo BT?
			return ((r_val > min(l_val_n,l_val) && r_val < max(l_val_n,l_val)) || (r_val > min(l_val_p,l_val) && r_val < max(l_val_p,l_val))) ? 0 : abs(l_val-r_val);
		}

		Array2D<uchar>::Data l;
		Array2D<uchar>::Data r;
	};
}

class AbsDiffBT : public DSBase<impl::AbsDiffBT> {
public:
	typedef impl::AbsDiffBT DataType;
	typedef unsigned short Type;

	AbsDiffBT() : DSBase<DataType>(0, 0, 0, 0) {};
	AbsDiffBT(int width, int height, int disp_min, int disp_max)
		: DSBase<DataType>(width, height, disp_min, disp_max) {};

	void set(const Array2D<uchar>& l, const Array2D<uchar>& r) {
		data().l = l.data();
		data().r = r.data();
	}

	static constexpr short COST_MAX = 255;
};

#endif
