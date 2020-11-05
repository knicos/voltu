#ifndef _CUDATL_MEMORY_HPP_
#define _CUDATL_MEMORY_HPP_

#include <cudatl/host_utility.hpp>

namespace cudatl
{

template <typename T>
T *allocate(size_t size)
{
	T *ptr;
	cudatl::safeCall(cudaMalloc(&ptr, size*sizeof(T)));
	return ptr;
}

template <typename T>
T *allocate(size_t width, size_t height, uint &pitch)
{
	if (width == 1 || height == 1)
	{
		pitch = width;
		return allocateMemory<T>((width > height) ? width : height);
	}
	else
	{
		T *ptr;
		size_t ptmp;
		cudatl::safeCall(cudaMallocPitch(&ptr, &ptmp, width*sizeof(T), height));
		pitch = ptmp/sizeof(T);
		return ptr;
	}
}

template <typename T>
void free(T *ptr)
{
	cudatl::safeCall(cudaFree(ptr));
}

}

#endif
