/**
 * Sum of absolute differences matching cost implemented using integral images.
 */

#ifndef _FTL_LIBSTEREO_COSTS_SAD_HPP_
#define _FTL_LIBSTEREO_COSTS_SAD_HPP_

#include <opencv2/core/core.hpp>
#include "array2d.hpp"
#include "dsbase.hpp"

#include <cuda_runtime.h>

namespace impl {
	struct SADMatchingCost : DSImplBase<float> {
		typedef float Type;

		SADMatchingCost(ushort w, ushort h, ushort dmin, ushort dmax) : DSImplBase<float>({w,h,dmin,dmax}), ww(0), wh(0) {}

		__host__ __device__ inline float operator()(const int y, const int x, const int d) const {
			if ((x-d-ww-1) < 0) { return COST_MAX; }
			if ((x+ww) >= width) { return COST_MAX; }
			if ((y-wh-1) < 0) { return COST_MAX; }
			if ((y+wh) >= height) { return COST_MAX; }

			int wl = l(y+wh,x+ww) + l(y-wh-1,x-ww-1)
						- l(y+wh, x-ww-1) - l(y-wh-1, x+ww);

			int wr = r(y+wh,x-d+ww) + r(y-wh-1,x-d-ww-1)
						- r(y+wh, x-d-ww-1) - r(y-wh-1, x-d+ww);

			return abs(wl - wr)/float((2*ww+1)*(2*wh+1));
		}

		static constexpr Type COST_MAX = 255*3;

		Array2D<int32_t>::Data l;
		Array2D<int32_t>::Data r;
		uchar ww;
		uchar wh;
	};
}

class SADMatchingCost : public DSBase<impl::SADMatchingCost> {
public:
	typedef impl::SADMatchingCost DataType;
	typedef float Type;

	SADMatchingCost() : DSBase<DataType>(0, 0, 0, 0) {};
	SADMatchingCost(int width, int height, int disp_min, int disp_max)
		: DSBase<DataType>(width, height, disp_min, disp_max) {
			setWindow(0, 0);
	}

	void set(cv::InputArray l, cv::InputArray r);

	void setWindow(int w, int h) {
		data().ww = w/2;
		data().wh = h/2;
	}

	static constexpr Type COST_MAX = DataType::COST_MAX;

protected:
	Array2D<int32_t> l_;
	Array2D<int32_t> r_;
};

#endif
