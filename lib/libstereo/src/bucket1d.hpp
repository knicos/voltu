#ifndef _FTL_LIBSTEREO_BUCKET1D_HPP_
#define _FTL_LIBSTEREO_BUCKET1D_HPP_

#include "memory.hpp"

template<typename T, int DEPTH>
class Bucket1D {
public:
	Bucket1D() : width(0), needs_free_(false) {
		data_.buckets = nullptr;
		data_.counters = nullptr;
	}

	Bucket1D(int w) : width(w), needs_free_(true) {
		data_.counters = allocateMemory<uint>(w);
		data_.buckets = allocateMemory2D<T>(DEPTH, w, data_.bucket_pitch);
	}

	~Bucket1D() {
		free();
	}

	void free() {
		if (needs_free_ && data_.buckets) {
			freeMemory(data_.counters);
			freeMemory(data_.buckets);
		}
	}

	Bucket1D<T, DEPTH> &operator=(const Bucket1D<T, DEPTH> &c) {
		data_ = c.data_;
		width = c.width;
		needs_free_ = false;
		return *this;
	}

	struct Data {
		__host__ __device__ inline uint& operator() (const int y) {
			return counters[y];
		}

		__host__ __device__ inline const uint& operator() (const int y) const {
			return counters[y];
		}

		__host__ __device__ inline const T& operator() (const int y, int d) const {
			return buckets[d + y*bucket_pitch];
		}

		__host__ __device__ inline void add (const int y, T v) {
			uint ix = atomicInc(&counters[y], DEPTH);
			buckets[ix + y*bucket_pitch] = v;
		}

		__host__ __device__ inline const T *ptr(const int y) const { return &buckets[y*bucket_pitch]; }

		uint bucket_pitch;
		T *buckets;
		uint *counters;
	};

	void create(int w) {
		if (w == width) return;
		width = w;
		free();
		needs_free_ = true;
		data_.counters = allocateMemory<uint>(w);
		data_.buckets = allocateMemory2D<T>(DEPTH, w, data_.bucket_pitch);
	}

	inline Data &data() { return data_; }
	inline const Data &data() const { return data_; }

	void toMat(cv::Mat &m) {
		#ifdef USE_GPU
		cv::cuda::GpuMat gm;
		toGpuMat(gm);
		gm.download(m);
		#else
		m = cv::Mat(1, width, cv::traits::Type<int>::value, data_.counters);
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
		return cv::Mat(1, width, cv::traits::Type<int>::value, data_.counters);
		#endif
	}

	const cv::Mat toMat() const {
		#ifdef USE_GPU
		cv::cuda::GpuMat gm(1, width, cv::traits::Type<int>::value, (void*)data_.counters);
		cv::Mat m;
		gm.download(m);
		return m;
		#else
		return cv::Mat(height, width, cv::traits::Type<int>::value, data_.counters);
		#endif
	}

	void toGpuMat(cv::cuda::GpuMat &m) {
		#ifdef USE_GPU
		m = cv::cuda::GpuMat(1, width, cv::traits::Type<int>::value, (void*)data_.counters);
		#else
		// TODO
		#endif
	}

	cv::cuda::GpuMat toGpuMat() {
		#ifdef USE_GPU
		return cv::cuda::GpuMat(1, width, cv::traits::Type<int>::value, (void*)data_.counters);
		#else
		return cv::cuda::GpuMat(1, width, cv::traits::Type<int>::value);
		#endif
	}

	int width;

private:
	Data data_;
	bool needs_free_;
};

#endif
