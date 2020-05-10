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
		typedef typename A::Type Type;

		DualCosts(ushort w, ushort h, ushort dmin, ushort dmax) : DSImplBase<Type>({w,h,dmin,dmax}),
			cost_a(w,h,dmin,dmax), cost_b(w,h,dmin,dmax) {}

		__host__ __device__ inline Type operator()(const int y, const int x, const int d) const {
			return cost_a(y,x,d) + cost_b(y,x,d);
		}

		A cost_a;
		B cost_b;
		static constexpr Type COST_MAX = A::COST_MAX + B::COST_MAX;
	};

	template <typename A, int N>
	struct MultiCosts : DSImplBase<typename A::Type> {
		typedef typename A::Type Type;

		MultiCosts(ushort w, ushort h, ushort dmin, ushort dmax) : DSImplBase<Type>({w,h,dmin,dmax}) {}

		__host__ __device__ inline Type operator()(const int y, const int x, const int d) const {
			Type cost = 0;
			#pragma unroll
			for (int n=0; n<N; ++n) {
				cost += costs[n](y,x,d);
			}
			return cost;
		}

		A costs[N];
		static constexpr Type COST_MAX = A::COST_MAX * N;
	};

	template <typename A, int N>
	struct MultiCostsWeighted : DSImplBase<typename A::Type> {
		typedef typename A::Type Type;

		MultiCostsWeighted(ushort w, ushort h, ushort dmin, ushort dmax) : DSImplBase<Type>({w,h,dmin,dmax}) {}

		__host__ __device__ inline Type operator()(const int y, const int x, const int d) const {
			float cost = 0;
			float pw = 1.0f;
			#pragma unroll
			for (int n=0; n<N; ++n) {
				float w = weights[n](y,x);
				cost += pw*w*float(costs[n](y,x,d));
				pw *= 1.0f-w;
			}
			return round(cost);
		}

		A costs[N];
		Array2D<float>::Data weights[N];
		static constexpr Type COST_MAX = A::COST_MAX;
	};

	template <typename A, typename B>
	struct DualCostsWeighted : DSImplBase<typename A::Type> {
		static_assert(std::is_same<typename A::Type, typename B::Type>::value, "Cost types must be the same");
		typedef typename A::Type Type;

		DualCostsWeighted(ushort w, ushort h, ushort dmin, ushort dmax) : DSImplBase<Type>({w,h,dmin,dmax}),
			cost_a(w,h,dmin,dmax), cost_b(w,h,dmin,dmax) {}

		__host__ __device__ inline Type operator()(const int y, const int x, const int d) const {
			const float w = max(weights_l(y,x), weights_r(y,x));
			return cost_a(y,x,d)*w + cost_b(y,x,d)*(1.0f - w);
		}

		A cost_a;
		B cost_b;
		Array2D<float>::Data weights_l;
		Array2D<float>::Data weights_r;
		static constexpr Type COST_MAX = A::COST_MAX + B::COST_MAX;
	};

}


template <typename A, typename B>
class DualCosts : public DSBase<impl::DualCosts<typename A::DataType,typename B::DataType>> {
public:
	typedef impl::DualCosts<typename A::DataType,typename B::DataType> DataType;
	typedef typename A::DataType::Type Type;

	DualCosts(int width, int height, int disp_min, int disp_max, A &a, B &b)
		: DSBase<DataType>(width, height, disp_min, disp_max),
			cost_a(a), cost_b(b) {};

	void set() {
		this->data().cost_a = cost_a.data();
		this->data().cost_b = cost_b.data();
	}

	static constexpr Type COST_MAX = A::COST_MAX + B::COST_MAX;

protected:
	A &cost_a;
	B &cost_b;
};

template <typename A, int N>
class MultiCosts : public DSBase<impl::MultiCosts<typename A::DataType,N>> {
public:
	typedef impl::MultiCosts<typename A::DataType,N> DataType;
	typedef typename A::DataType::Type Type;

	MultiCosts(int width, int height, int disp_min, int disp_max)
		: DSBase<DataType>(width, height, disp_min, disp_max) {};

	void add(int n, A &c) {
		if (n >= N || n < 0) throw std::exception();
		costs[n] = &c;
	}

	void set() {
		for (int n=0; n<N; ++n) {
			this->data().costs[n] = costs[n]->data();
		}
	}

	static constexpr Type COST_MAX = A::COST_MAX * N;

protected:
	A *costs[N];
};

template <typename A, int N>
class MultiCostsWeighted : public DSBase<impl::MultiCostsWeighted<typename A::DataType,N>> {
public:
	typedef impl::MultiCostsWeighted<typename A::DataType,N> DataType;
	typedef typename A::DataType::Type Type;

	MultiCostsWeighted(int width, int height, int disp_min, int disp_max)
		: DSBase<DataType>(width, height, disp_min, disp_max) {};

	void add(int n, A &c, Array2D<float> &w) {
		if (n >= N || n < 0) throw std::exception();
		costs[n] = &c;
		weights[n] = &w;
	}

	void set() {
		for (int n=0; n<N; ++n) {
			this->data().costs[n] = costs[n]->data();
			this->data().weights[n] = weights[n]->data();
		}
	}

	static constexpr Type COST_MAX = A::COST_MAX;

protected:
	A *costs[N];
	Array2D<float> *weights[N];
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
	typedef typename A::DataType::Type Type;

	DualCostsWeighted(int width, int height, int disp_min, int disp_max, A &a, B &b, Array2D<float> &weightsl, Array2D<float> &weightsr)
		: DSBase<DataType>(width, height, disp_min, disp_max),
			cost_a(a), cost_b(b), weights_l(weightsl), weights_r(weightsr) {};

	void set() {
		this->data().cost_a = cost_a.data();
		this->data().cost_b = cost_b.data();
		this->data().weights_l = weights_l.data();
		this->data().weights_r = weights_r.data();
	}

	static constexpr Type COST_MAX = A::DataType::COST_MAX + B::DataType::COST_MAX;

protected:
	A &cost_a;
	B &cost_b;
	Array2D<float> &weights_l;
	Array2D<float> &weights_r;
};

#endif
