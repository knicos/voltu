#ifndef _FTL_LIBSTEREO_COSTS_DUAL_HPP_
#define _FTL_LIBSTEREO_COSTS_DUAL_HPP_

#include <opencv2/core/core.hpp>
#include "array2d.hpp"
#include "dsbase.hpp"

#include <cuda_runtime.h>

namespace impl {
	template <typename A, typename B>
	struct DualCosts : DSImplBase<typename A::Type> {
		static_assert(std::is_same<typename A::Type, typename B::Type>::value, "Cost types must be the same");
		typedef unsigned short Type;

		DualCosts(ushort w, ushort h, ushort dmin, ushort dmax) : DSImplBase<unsigned short>({w,h,dmin,dmax}),
			cost_a(w,h,dmin,dmax), cost_b(w,h,dmin,dmax) {}

		__host__ __device__ inline unsigned short operator()(const int y, const int x, const int d) const {
			return cost_a(y,x,d) + cost_b(y,x,d);
		}

		A cost_a;
		B cost_b;
	};

	template <typename A, typename B>
	struct DualCostsWeighted : DSImplBase<typename A::Type> {
		static_assert(std::is_same<typename A::Type, typename B::Type>::value, "Cost types must be the same");
		typedef unsigned short Type;

		DualCostsWeighted(ushort w, ushort h, ushort dmin, ushort dmax) : DSImplBase<unsigned short>({w,h,dmin,dmax}),
			cost_a(w,h,dmin,dmax), cost_b(w,h,dmin,dmax) {}

		__host__ __device__ inline unsigned short operator()(const int y, const int x, const int d) const {
			const float w = weights_l(y,x);
			return cost_a(y,x,d)*w + cost_b(y,x,d)*(1.0f - w);
		}

		A cost_a;
		B cost_b;
		Array2D<float>::Data weights_l;
		Array2D<float>::Data weights_r;
	};
}


template <typename A, typename B>
class DualCosts : public DSBase<impl::DualCosts<typename A::DataType,typename B::DataType>> {
public:
	typedef impl::DualCosts<typename A::DataType,typename B::DataType> DataType;
	typedef unsigned short Type;

	DualCosts(int width, int height, int disp_min, int disp_max, A &a, B &b)
		: DSBase<DataType>(width, height, disp_min, disp_max),
			cost_a(a), cost_b(b) {};

	void set() {
		this->data().cost_a = cost_a.data();
		this->data().cost_b = cost_b.data();
	}

	static constexpr short COST_MAX = A::COST_MAX + B::COST_MAX;

protected:
	A &cost_a;
	B &cost_b;
};

/**
 * Combine two cost functions with weight. Weight in Array2D<float> same size
 * as input. Weight values assumed in range [0,1] and cost is calculated as
 *
 *    a(y,x)*w(y,x) + b(y,x)*(1.0-w(y,x))
 */
template <typename A, typename B>
class DualCostsWeighted : public DSBase<impl::DualCostsWeighted<typename A::DataType,typename B::DataType>> {
public:
	typedef impl::DualCostsWeighted<typename A::DataType,typename B::DataType> DataType;
	typedef unsigned short Type;

	DualCostsWeighted(int width, int height, int disp_min, int disp_max, A &a, B &b, Array2D<float> &weightsl, Array2D<float> &weightsr)
		: DSBase<DataType>(width, height, disp_min, disp_max),
			cost_a(a), cost_b(b), weights_l(weightsl), weights_r(weightsr) {};

	void set() {
		this->data().cost_a = cost_a.data();
		this->data().cost_b = cost_b.data();
		this->data().weights_l = weights_l.data();
		this->data().weights_r = weights_r.data();
	}

	static constexpr short COST_MAX = A::DataType::COST_MAX + B::DataType::COST_MAX;

protected:
	A &cost_a;
	B &cost_b;
	Array2D<float> &weights_l;
	Array2D<float> &weights_r;
};

#endif
