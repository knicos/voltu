#ifndef _FTL_LIBSTEREO_UTIL_HPP_
#define _FTL_LIBSTEREO_UTIL_HPP_

#include <cuda_runtime.h>

#ifdef USE_GPU
#define __cuda__ __host__ __device__
#else
#define __cuda__
#endif

static constexpr int WARP_SIZE = 32;
static constexpr unsigned int FULL_MASK = 0xFFFFFFFF;

template <typename T>
__device__ inline T warpMin(T e) {
	for (int i = WARP_SIZE/2; i > 0; i /= 2) {
		const T other = __shfl_xor_sync(FULL_MASK, e, i, WARP_SIZE);
		e = min(e, other);
	}
	return e;
}

#ifdef USE_GPU

template <typename FUNCTOR>
__global__ void kernel2D(FUNCTOR f, ushort2 size) {
	const ushort2 thread{ushort(threadIdx.x+blockIdx.x*blockDim.x), ushort(threadIdx.y+blockIdx.y*blockDim.y)};
	const ushort2 stride{ushort(blockDim.x * gridDim.x), ushort(blockDim.y * gridDim.y)};
	f(thread, stride, size);
}

template <typename FUNCTOR>
__global__ void kernel1D(FUNCTOR f, ushort size) {
	const ushort thread = threadIdx.x+blockIdx.x*blockDim.x;
	const ushort stride = blockDim.x * gridDim.x;
	f(thread, stride, size);
}

#endif

/**
 * Wrapper to initiate a parallel processing job using a given functor. The
 * width and height parameters are used to determine the number of threads.
 */
template <typename FUNCTOR>
void parallel1D(const FUNCTOR &f, int size) {
#if __cplusplus >= 201703L
	static_assert(std::is_invocable<FUNCTOR,int,int,int>::value, "Parallel1D requires a valid functor: void()(int,int,int)");
#endif

#ifdef USE_GPU
	cudaSafeCall(cudaGetLastError());
	kernel1D<<<64, 256, 0, 0>>>(f,size);
	cudaSafeCall(cudaGetLastError());
	//cudaSafeCall(cudaDeviceSynchronize());
#else
	static constexpr int MAX_THREADS=32;
	int stride = MAX_THREADS;
	FUNCTOR f2 = f;  // Make a copy
	#pragma omp parallel for
	for (int i=0; i<MAX_THREADS; ++i) {
		f2(i, stride, size);
	}
#endif
}


/**
 * Wrapper to initiate a parallel processing job using a given functor. The
 * width and height parameters are used to determine the number of threads.
 */
template <typename FUNCTOR>
void parallel1DWarp(const FUNCTOR &f, int size, int size2) {
#if __cplusplus >= 201703L
	//static_assert(std::is_invocable<FUNCTOR,int,int,int>::value, "Parallel1D requires a valid functor: void()(int,int,int)");
	// Is this static_assert correct?
	static_assert(std::is_invocable<FUNCTOR,ushort2,ushort2,ushort2>::value, "Parallel1D requires a valid functor: void()(ushort2,ushort2,ushort2)");
#endif

#ifdef USE_GPU
	cudaSafeCall(cudaGetLastError());
	const dim3 gridSize(1, (size+8-1) / 8);
	const dim3 blockSize(32, 8);
	ushort2 tsize{ushort(size2), ushort(size)};
	kernel2D<<<gridSize, blockSize, 0, 0>>>(f,tsize);
	cudaSafeCall(cudaGetLastError());
	//cudaSafeCall(cudaDeviceSynchronize());
#else
	static constexpr int MAX_THREADS=32;
	FUNCTOR f2 = f;  // Make a copy
	#pragma omp parallel for
	for (int i=0; i<MAX_THREADS; ++i) {
		ushort2 thread{0,ushort(i)};
		ushort2 stride{1, MAX_THREADS}; //(height+MAX_THREADS-1)/MAX_THREADS};
		ushort2 tsize{ushort(size2), ushort(size)};
		f2(thread, stride, tsize);
	}
#endif
}

/**
 * Wrapper to initiate a parallel processing job using a given functor. The
 * width and height parameters are used to determine the number of threads.
 */
template <typename FUNCTOR>
void parallel2D(const FUNCTOR &f, int width, int height) {
	//static_assert(std::is_function<FUNCTOR>::value, "Parallel2D requires a valid functor");
#if __cplusplus >= 201703L
	static_assert(std::is_invocable<FUNCTOR,ushort2,ushort2,ushort2>::value, "Parallel2D requires a valid functor: void()(ushort2,ushort2,ushort2)");
#endif
	assert(width > 0 && width <= 4096);
	assert(height > 0 && height <= 4096);

#ifdef USE_GPU
	static constexpr uint T_PER_BLOCK=8;
	const dim3 gridSize((width + T_PER_BLOCK - 1)/T_PER_BLOCK, (height + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);
	ushort2 size{ushort(width), ushort(height)};
	kernel2D<<<gridSize, blockSize, 0, 0>>>(f,size);
	cudaSafeCall(cudaGetLastError());
#else
	static constexpr int MAX_THREADS=32;
	FUNCTOR f2 = f;  // Make a copy
	#pragma omp parallel for
	for (int i=0; i<MAX_THREADS; ++i) {
		ushort2 thread{0,ushort(i)};
		ushort2 stride{1, MAX_THREADS}; //(height+MAX_THREADS-1)/MAX_THREADS};
		ushort2 size{ushort(width), ushort(height)};
		f2(thread, stride, size);
	}
#endif
}

#endif
