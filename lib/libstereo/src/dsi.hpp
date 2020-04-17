#ifndef _FTL_LIBSTEREO_DSI_HPP_
#define _FTL_LIBSTEREO_DSI_HPP_

#include "dsbase.hpp"
#include "memory.hpp"
#include <opencv2/core/mat.hpp>

////////////////////////////////////////////////////////////////////////////////



namespace impl {
	// Basic data class for use on GPU and CPU
	template <typename T>
	struct DSI : DSImplBase<T> {
		typedef T Type;

		DSI() : data(nullptr) {}
		DSI(ushort w, ushort h, ushort dmin, ushort dmax) : DSImplBase<T>({w,h,dmin,dmax}), data(nullptr) {}

		__host__ __device__ inline T& operator() (const int y, const int x, const int d) {
			return data[d-this->disp_min + x*this->disparityRange() + y*this->width*this->disparityRange()];
		}

		__host__ __device__ inline const T& operator() (const int y, const int x, const int d) const {
			return data[d-this->disp_min + x*this->disparityRange() + y*this->width*this->disparityRange()];
		}

		uint pitch;
		T *data;
	};
}

template<typename T>
class DisparitySpaceImage : public DSBase<impl::DSI<T>> {
public:
	typedef impl::DSI<T> DataType;
	typedef T Type;

	DisparitySpaceImage();
	DisparitySpaceImage(int width, int height, int disp_min, int disp_max);
	~DisparitySpaceImage();

	DisparitySpaceImage(const DisparitySpaceImage<T> &)=delete;

	T operator()(int y, int x, int d) const;

	void create(int w, int h, int dmin, int dmax);

	void clear();

	void set(const T& v);

	template<typename A>
	void add(const A &other, double scale=1.0);

	void mul(double a);

	cv::Mat Mat();

private:
	//DataType data_;
	bool needs_free_;
};

#include "dsi_impl.hpp"

// Common types
typedef DisparitySpaceImage<unsigned char> DSImage8U;
typedef DisparitySpaceImage<unsigned short> DSImage16U;
typedef DisparitySpaceImage<unsigned int> DSImage32U;
typedef DisparitySpaceImage<float> DSImageFloat;

#endif
