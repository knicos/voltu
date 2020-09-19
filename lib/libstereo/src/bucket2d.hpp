#ifndef _FTL_LIBSTEREO_BUCKET2D_HPP_
#define _FTL_LIBSTEREO_BUCKET2D_HPP_

#include "memory.hpp"

template<typename T, int DEPTH>
class Bucket2D {
public:
	Bucket2D() : width(0), height(0), needs_free_(false) {
		data_.buckets = nullptr;
		data_.counters = nullptr;
	}

	Bucket2D(int w, int h) : width(w), height(h), needs_free_(true) {
		data_.counters = allocateMemory2D<uint>(w, h, data_.counter_pitch);
		data_.buckets = allocateMemory2D<T>(w*DEPTH, h, data_.bucket_pitch);
	}

	~Bucket2D() {
		free();
	}

	void free() {
		if (needs_free_ && data_.buckets) {
			freeMemory(data_.counters);
			freeMemory(data_.buckets);
		}
	}

	Bucket2D<T, DEPTH> &operator=(const Bucket2D<T, DEPTH> &c) {
		data_ = c.data_;
		width = c.width;
		height = c.height;
		needs_free_ = false;
		return *this;
	}

	struct Data {
		__host__ __device__ inline uint& operator() (const int y, const int x) {
			return counters[x + y*counter_pitch];
		}

		__host__ __device__ inline const uint& operator() (const int y, const int x) const {
			return counters[x + y*counter_pitch];
		}

		__host__ __device__ inline const T& operator() (const int y, const int x, int d) const {
			return buckets[d + x*DEPTH + y*bucket_pitch];
		}

		__host__ __device__ inline void add (const int y, const int x, T v) {
			uint ix = atomicInc(&counters[x + y*counter_pitch], DEPTH);
			buckets[ix + x*DEPTH + y*bucket_pitch] = v;
		}

		__host__ __device__ inline uint *ptr(const int y) { return &counters[y*counter_pitch]; }
		__host__ __device__ inline const uint *ptr(const int y) const { return &counters[y*counter_pitch]; }
		__host__ __device__ inline const T *ptr(const int y, const int x) const { return &buckets[x*DEPTH+y*bucket_pitch]; }

		uint bucket_pitch;
		uint counter_pitch;
		T *buckets;
		uint *counters;
	};

	void create(int w, int h) {
		if (w == width && h == height) return;
		width = w;
		height = h;
		free();
		needs_free_ = true;
		data_.counters = allocateMemory2D<uint>(w, h, data_.counter_pitch);
		data_.buckets = allocateMemory2D<T>(w*DEPTH, h, data_.bucket_pitch);
	}

	inline Data &data() { return data_; }
	inline const Data &data() const { return data_; }

	void toMat(cv::Mat &m) {
		#ifdef USE_GPU
		cv::cuda::GpuMat gm;
		toGpuMat(gm);
		gm.download(m);
		#else
		m = cv::Mat(height, width, cv::traits::Type<int>::value, data_.counters);
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
		return cv::Mat(height, width, cv::traits::Type<int>::value, data_.counters);
		#endif
	}

	const cv::Mat toMat() const {
		#ifdef USE_GPU
		cv::cuda::GpuMat gm(height, width, cv::traits::Type<int>::value, (void*)data_.counters, size_t(size_t(data_.counter_pitch)*sizeof(uint)));
		cv::Mat m;
		gm.download(m);
		return m;
		#else
		return cv::Mat(height, width, cv::traits::Type<int>::value, data_.counters);
		#endif
	}

	void toGpuMat(cv::cuda::GpuMat &m) {
		#ifdef USE_GPU
		m = cv::cuda::GpuMat(height, width, cv::traits::Type<int>::value, (void*)data_.counters, size_t(size_t(data_.counter_pitch)*sizeof(uint)));
		#else
		// TODO
		#endif
	}

	cv::cuda::GpuMat toGpuMat() {
		#ifdef USE_GPU
		return cv::cuda::GpuMat(height, width, cv::traits::Type<int>::value, (void*)data_.counters, size_t(size_t(data_.counter_pitch)*sizeof(uint)));
		#else
		return cv::cuda::GpuMat(height, width, cv::traits::Type<int>::value);
		#endif
	}

	int width;
	int height;

private:
	Data data_;
	bool needs_free_;
};

#endif
