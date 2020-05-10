#ifndef _FTL_LIBSTEREO_COSTS_EXP_HPP_
#define _FTL_LIBSTEREO_COSTS_EXP_HPP_

#include "array2d.hpp"
#include "dsbase.hpp"

#include <cuda_runtime.h>

namespace impl {
	template <typename A>
	struct ExpCost : DSImplBase<float> {
		typedef float Type;

		ExpCost(ushort w, ushort h, ushort dmin, ushort dmax) : DSImplBase<float>({w,h,dmin,dmax}),
			cost_a(w,h,dmin,dmax) {}

		__host__ __device__ inline float operator()(const int y, const int x, const int d) const {
			return 1.0f-expf(-cost_a(y,x,d)*l);
		}

		A cost_a;
		float l=1.0;
		static constexpr Type COST_MAX = A::COST_MAX;
	};

	template <typename A>
	struct LinearCost : DSImplBase<float> {
		typedef float Type;

		LinearCost(ushort w, ushort h, ushort dmin, ushort dmax) : DSImplBase<float>({w,h,dmin,dmax}),
			cost_a(w,h,dmin,dmax) {}

		__host__ __device__ inline float operator()(const int y, const int x, const int d) const {
			return cost_a(y,x,d)*s*a;
		}

		A cost_a;
		float a=1.0;
		float s=1.0;

		static constexpr Type COST_MAX = A::COST_MAX;
	};

	template <typename A, typename T>
	struct WeightedCost : DSImplBase<T> {
		typedef T Type;

		WeightedCost(ushort w, ushort h, ushort dmin, ushort dmax) : DSImplBase<Type>({w,h,dmin,dmax}),
			cost(w,h,dmin,dmax) {}

		__host__ __device__ inline Type operator()(const int y, const int x, const int d) const {
			const float w = max(weights_l(y,x), weights_r(y,x));
			return round(cost(y,x,d)*w);
		}

		A cost;
		Array2D<float>::Data weights_l;
		Array2D<float>::Data weights_r;
		static constexpr Type COST_MAX = A::COST_MAX;
	};
}

template <typename A>
class ExpCost : public DSBase<impl::ExpCost<typename A::DataType>> {
public:
	typedef impl::ExpCost<typename A::DataType> DataType;
	typedef float Type;

	ExpCost(int width, int height, int disp_min, int disp_max, A &a, float l)
		: DSBase<DataType>(width, height, disp_min, disp_max),
			cost_a(a) {};

	void set() {
		this->data().cost_a = cost_a.data();
	}

	void set(float l) {
		this->data().cost_a = cost_a.data();
		this->data().l = l;
	}

	static constexpr float COST_MAX = A::COST_MAX;

protected:
	A &cost_a;
};

template <typename A>
class LinearCost : public DSBase<impl::LinearCost<typename A::DataType>> {
public:
	typedef impl::LinearCost<typename A::DataType> DataType;
	typedef float Type;

	LinearCost(int width, int height, int disp_min, int disp_max, A &a, float l)
		: DSBase<DataType>(width, height, disp_min, disp_max),
			cost_a(a) {

		this->data().s = l;
		this->data().a = 1.0f/COST_MAX;
	}

	void set() {
		this->data().cost_a = cost_a.data();
	}

	void set(float s) {
		this->data().cost_a = cost_a.data();
		this->data().s = s;
	}

	static const typename A::Type COST_MAX = A::COST_MAX;

protected:
	A &cost_a;
};


template <typename A, typename T=unsigned short>
class WeightedCost : public DSBase<impl::WeightedCost<typename A::DataType, T>> {
public:
	typedef impl::WeightedCost<typename A::DataType, T> DataType;
	typedef T Type;

	WeightedCost(int width, int height, int disp_min, int disp_max, A &a, Array2D<float> &wl, Array2D<float> &wr)
		: DSBase<DataType>(width, height, disp_min, disp_max),
			cost(&a), weights_l(&wl), weights_r(&wr) {

	}

	WeightedCost(int width, int height, int disp_min, int disp_max)
		: DSBase<DataType>(width, height, disp_min, disp_max),
			cost(nullptr), weights_l(nullptr), weights_r(nullptr) {

	}

	void setCost(A &c) { cost = &c; }

	void setWeights(Array2D<float> &wl, Array2D<float> &wr) {
		weights_l = &wl;
		weights_r = &wr;
	}

	void set() {
		this->data().cost = cost->data();
		this->data().weights_l = weights_l->data();
		this->data().weights_r = weights_r->data();
	}

	static const T COST_MAX = A::COST_MAX;

protected:
	Array2D<float> *weights_l;
	Array2D<float> *weights_r;
	A *cost;
};

#endif
