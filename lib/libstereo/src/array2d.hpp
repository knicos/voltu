#ifndef _FTL_LIBSTEREO_ARRAY2D_HPP_
#define _FTL_LIBSTEREO_ARRAY2D_HPP_

#include "memory.hpp"

template<typename T>
class Array2D {
public:
	Array2D() : width(0), height(0), needs_free_(false) {
		data_.data = nullptr;
	}

	Array2D(int w, int h) : width(w), height(h), needs_free_(true) {
		data_.data = allocateMemory2D<T>(w, h, data_.pitch);
	}

	explicit Array2D(cv::Mat &m) : needs_free_(false) {
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
	}

	~Array2D() {
		free();
	}

	void free() {
		if (needs_free_ && data_.data) freeMemory(data_.data);
	}

	Array2D<T> &operator=(const Array2D<T> &c) {
		data_ = c.data_;
		width = c.width;
		height = c.height;
		needs_free_ = false;
		return *this;
	}

	struct Data {
		__host__ __device__ inline T& operator() (const int y, const int x) {
			return data[x + y*pitch];
		}

		__host__ __device__ inline const T& operator() (const int y, const int x) const {
			return data[x + y*pitch];
		}

		__host__ __device__ inline T *ptr(const int y) { return &data[y*pitch]; }
		__host__ __device__ inline const T *ptr(const int y) const { return &data[y*pitch]; }

		uint pitch;
		T *data;
	};

	void create(int w, int h) {
		if (w == width && h == height) return;
		width = w;
		height = h;
		free();
		needs_free_ = true;
		data_.data = allocateMemory2D<T>(w, h, data_.pitch);
	}

	inline Data &data() { return data_; }
	inline const Data &data() const { return data_; }

	void toMat(cv::Mat &m) {
		#ifdef USE_GPU
		cv::cuda::GpuMat gm;
		toGpuMat(gm);
		gm.download(m);
		#else
		m = cv::Mat(height, width, cv::traits::Type<T>::value, data_.data);
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
		return cv::Mat(height, width, cv::traits::Type<T>::value, data_.data);
		#endif
	}

	const cv::Mat toMat() const {
		#ifdef USE_GPU
		cv::cuda::GpuMat gm(height, width, cv::traits::Type<T>::value, (void*)data_.data, size_t(size_t(data_.pitch)*sizeof(T)));
		cv::Mat m;
		gm.download(m);
		return m;
		#else
		return cv::Mat(height, width, cv::traits::Type<T>::value, data_.data);
		#endif
	}

	void toGpuMat(cv::cuda::GpuMat &m) {
		#ifdef USE_GPU
		m = cv::cuda::GpuMat(height, width, cv::traits::Type<T>::value, (void*)data_.data, size_t(size_t(data_.pitch)*sizeof(T)));
		#else
		// TODO
		#endif
	}

	cv::cuda::GpuMat toGpuMat() {
		#ifdef USE_GPU
		return cv::cuda::GpuMat(height, width, cv::traits::Type<T>::value, (void*)data_.data, size_t(size_t(data_.pitch)*sizeof(T)));
		#else
		return cv::cuda::GpuMat(height, width, cv::traits::Type<T>::value);
		#endif
	}

	int width;
	int height;

private:
	Data data_;
	bool needs_free_;
};

#endif
