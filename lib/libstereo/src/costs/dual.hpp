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
			return (cost_a(y, x, d) + cost_b(y, x, d));
		}

		A cost_a;
		B cost_b;
	};
}

template <typename A, typename B>
class DualCosts : public DSBase<impl::DualCosts<typename A::DataType,typename B::DataType>> {
public:
	typedef impl::DualCosts<typename A::DataType,typename B::DataType> DataType;
	typedef unsigned short Type;

	//DualCosts() : DSBase<DataType>(0, 0, 0, 0) {};
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

#endif
