#ifndef _FTL_LIBSTEREO_MEMORY_HPP_
#define _FTL_LIBSTEREO_MEMORY_HPP_

#include <cuda_runtime.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/cuda/common.hpp>

#ifdef USE_GPU
using OpenCVMat = cv::cuda::GpuMat;
#else
using OpenCVMat = cv::Mat;
#endif

template <typename T>
T *allocateMemory(size_t size) {
#ifdef USE_GPU
	T *ptr;
	cudaSafeCall(cudaMalloc(&ptr, size*sizeof(T)));
	return ptr;
#else
	return new T[size];
#endif
}

template <typename T>
T *allocateMemory2D(size_t width, size_t height, uint &pitch) {
	if (width == 1 || height == 1) {
		pitch = width;
		return allocateMemory<T>((width > height) ? width: height);
	} else {
	#ifdef USE_GPU
		T *ptr;
		size_t ptmp;
		cudaSafeCall(cudaMallocPitch(&ptr, &ptmp, width*sizeof(T), height));
		pitch = ptmp/sizeof(T);
		return ptr;
	#else
		pitch = width; //*sizeof(T);
		return new T[width*height];
	#endif
	}
}

template <typename T>
void freeMemory(T *ptr) {
#ifdef USE_GPU
	cudaSafeCall(cudaFree(ptr));
#else
	delete [] ptr;
#endif
}

#endif
