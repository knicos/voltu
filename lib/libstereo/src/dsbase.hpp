#ifndef _FTL_LIBSTEREO_DSBASE_HPP_
#define _FTL_LIBSTEREO_DSBASE_HPP_

#include <cuda_runtime.h>
#include <type_traits>

namespace impl {
	template <typename T>
	struct DSImplBase {
		typedef T Type;

		static_assert(std::is_arithmetic<T>::value, "Cost type is not numeric");

		//DSImplBase(int w, int h, int dmin, int dmax) : width(w), height(h), disp_min(dmin), disp_max(dmax) {}

		uint16_t width;
		uint16_t height;
		uint16_t disp_min;
		uint16_t disp_max;

		//static constexpr T COST_MAX = std::numeric_limits<T>::max();

		__host__ __device__ inline uint16_t disparityRange() const { return disp_max-disp_min+1; }
		__host__ __device__ inline uint32_t size() const { return width * disparityRange() * height; }
		__host__ __device__ inline uint32_t bytes() const { return size() * sizeof(Type); }
	};
}

/**
 * Base class representing a 3D disparity space structure. This may
 * implement the 3D data structure as a stored memory object or through
 * dynamic calculation using other data.
 */
template<typename T>
class DSBase {
	public:
	typedef T DataType;
	typedef typename T::Type Type;

#if __cplusplus >= 201703L
	static_assert(std::is_invocable_r<Type,T,int,int,int>::value, "Incorrect disparity space operator");
#endif
	static_assert(std::is_convertible<T, impl::DSImplBase<Type>>::value, "Data type is not derived from DSImplBase");

	DSBase(int w, int h, int dmin, int dmax) : data_(w,h,dmin,dmax) {}
	~DSBase() {};

	DataType &data() { return data_; };
	const DataType &data() const { return data_; };

	int width() const { return data_.width; }
	int height() const { return data_.height; }
	int numDisparities() const { return data_.disparityRange(); }
	int maxDisparity() const { return data_.disp_max; }
	int minDisparity() const { return data_.disp_min; }
	Type maxCost() const { return DataType::COST_MAX; }

	protected:
	DataType data_;
};

//#include "dsbase_impl.hpp"

#endif
