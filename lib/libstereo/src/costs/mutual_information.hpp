#ifndef _FTL_LIBSTEREO_COSTS_MI_HPP_
#define _FTL_LIBSTEREP_COSTS_MI_HPP_

#include <opencv2/core/core.hpp>
#include "array2d.hpp"
#include "dsbase.hpp"

#include <cuda_runtime.h>

namespace impl {
	template<typename T>
	struct MutualInformationMatchingCost : DSImplBase<T> {
		typedef unsigned short Type;

		MutualInformationMatchingCost(ushort w, ushort h, ushort dmin, ushort dmax) : DSImplBase<T>({w,h,dmin,dmax}) {}

		__host__ __device__ inline T operator()(const int y, const int x, const int d) const {
			if ((x-d) < 0) { return 0; }
			const int I1 = l(y,x);
			const int I2 = r(y,x-d);
			const float H1 = h1(0,I1);
			const float H2 = h2(0,I2);
			const float H12 = h12(I1,I2);
			return -(H1+H2-H12);
		}

		static constexpr T COST_MAX = 255;

		Array2D<unsigned char>::Data l;
		Array2D<unsigned char>::Data r;
		Array2D<float>::Data h1;
		Array2D<float>::Data h2;
		Array2D<float>::Data h12;
		float normalize = 1.0f;
	};
}

class MutualInformationMatchingCost : public DSBase<impl::MutualInformationMatchingCost<unsigned short>>{
public:
	typedef impl::MutualInformationMatchingCost<unsigned short> DataType;
	typedef unsigned short Type;

	MutualInformationMatchingCost(int width, int height, int disp_min, int disp_max) :
		DSBase<DataType>(width, height, disp_min, disp_max),
		l_(width, height), r_(width, height),
		h1_(256, 1), h2_(256, 1), h12_(256, 256)
		{
			data().l = l_.data();
			data().r = r_.data();
			data().h1 = h1_.data();
			data().h2 = h2_.data();
			data().h12 = h12_.data();
		}

	void set(const cv::Mat &left, const cv::Mat &right, const cv::Mat &disparity);

protected:
	Array2D<unsigned char> l_;
	Array2D<unsigned char> r_;
	Array2D<float> h1_;
	Array2D<float> h2_;
	Array2D<float> h12_;

	cv::Mat mapx_;
	cv::Mat mapy_;
	cv::Mat r_warp_;
};

#endif
