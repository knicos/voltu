#ifndef _FTL_LIBSTEREO_ARRAY1D_HPP_
#define _FTL_LIBSTEREO_ARRAY1D_HPP_

#include "memory.hpp"

template<typename T>
class Array1D {
public:
	Array1D() : width(0), needs_free_(false) {
		data_.data = nullptr;
	}

	Array1D(int w) : width(w), needs_free_(true) {
		data_.data = allocateMemory<T>(w);
	}

	/*explicit Array1D(cv::Mat &m) : needs_free_(false) {
		#ifdef USE_GPU
		create(m.cols, m.rows);
		cudaSafeCall(cudaMemcpy2D(data_.data, data_.pitch*sizeof(T), m.data, m.step, width*sizeof(T), height, cudaMemcpyHostToDevice));
		#else
		needs_free_ = false;
		data_.data = (T*)m.data;
		data_.pitch = m.step / sizeof(T);
		width = m.cols;
		height = m.rows;
		#endif
	}

	explicit Array2D(cv::cuda::GpuMat &m) : needs_free_(false) {
		#ifdef USE_GPU
		needs_free_ = false;
		data_.data = (T*)m.data;
		data_.pitch = m.step / sizeof(T);
		width = m.cols;
		height = m.rows;
		#else
		create(m.cols, m.rows);
		cudaSafeCall(cudaMemcpy2D(data_.data, data_.pitch*sizeof(T), m.data, m.step, width*sizeof(T), height, cudaMemcpyDeviceToHost));
		#endif
	}*/

	~Array1D() {
		free();
	}

	void free() {
		if (needs_free_ && data_.data) freeMemory(data_.data);
	}

	Array1D<T> &operator=(const Array1D<T> &c) {
		data_ = c.data_;
		width = c.width;
		needs_free_ = false;
		return *this;
	}

	struct Data {
		__host__ __device__ inline T& operator() (const int x) {
			return data[x];
		}

		__host__ __device__ inline const T& operator() (const int x) const {
			return data[x];
		}

		T *data;
	};

	void create(int w) {
		if (w == width) return;
		width = w;
		free();
		needs_free_ = true;
		data_.data = allocateMemory<T>(w);
	}

	inline Data &data() { return data_; }
	inline const Data &data() const { return data_; }

	void toMat(cv::Mat &m) {
		#ifdef USE_GPU
		cv::cuda::GpuMat gm;
		toGpuMat(gm);
		gm.download(m);
		#else
		m = cv::Mat(1, width, cv::traits::Type<T>::value, data_.data);
		#endif
	}

	cv::Mat toMat() {
		#ifdef USE_GPU
		cv::cuda::GpuMat gm;
		toGpuMat(gm);
		cv::Mat m;
		gm.download(m);
		return m;
		#else
		return cv::Mat(1, width, cv::traits::Type<T>::value, data_.data);
		#endif
	}

	const cv::Mat toMat() const {
		#ifdef USE_GPU
		cv::cuda::GpuMat gm(1, width, cv::traits::Type<T>::value, (void*)data_.data);
		cv::Mat m;
		gm.download(m);
		return m;
		#else
		return cv::Mat(1, width, cv::traits::Type<T>::value, data_.data);
		#endif
	}

	void toGpuMat(cv::cuda::GpuMat &m) {
		#ifdef USE_GPU
		m = cv::cuda::GpuMat(1, width, cv::traits::Type<T>::value, (void*)data_.data);
		#else
		// TODO
		#endif
	}

	cv::cuda::GpuMat toGpuMat() {
		#ifdef USE_GPU
		return cv::cuda::GpuMat(1, width, cv::traits::Type<T>::value, (void*)data_.data);
		#else
		return cv::cuda::GpuMat(1, width, cv::traits::Type<T>::value);
		#endif
	}

	int width;

private:
	Data data_;
	bool needs_free_;
};

#endif
