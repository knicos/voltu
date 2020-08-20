#ifndef _FTL_CUDA_COMMON_HPP_
#define _FTL_CUDA_COMMON_HPP_

#include <ftl/config.h>
#include <ftl/traits.hpp>

#if defined HAVE_CUDA

#include <ftl/cuda_util.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/cuda/common.hpp>
#include <ftl/cuda_half.hpp>
#include <ftl/cuda_texture.hpp>
#include <ftl/cuda_buffer.hpp>

#ifndef __CUDACC__
#include <exception>
#endif

/* Grid stride loop macros */
#define STRIDE_Y(I,N) int I = blockIdx.y * blockDim.y + threadIdx.y; I < N; I += blockDim.y * gridDim.y
#define STRIDE_X(I,N) int I = blockIdx.x * blockDim.x + threadIdx.x; I < N; I += blockDim.x * gridDim.x

void cudaCallback(cudaStream_t stream, const std::function<void()> &cb);

namespace ftl {
namespace cuda {

bool initialise();

bool hasCompute(int major, int minor);

int deviceCount();

int getDevice();

void setDevice(int);

void setDevice();

/**
 * Read a texture value using coordinates in the range of `b`, but from the
 * texture `a` which may have a different resolution.
 */
template <typename A, typename B>
__device__ inline A getScaledTex2D(int x, int y, ftl::cuda::TextureObject<A> &a, ftl::cuda::TextureObject<B> &b) {
	return a.tex2D(
		(int)((float)x * ((float)a.width() / (float)b.width())),
		(int)((float)y * ((float)a.height() / (float)b.height()))
	);
}

}
}

#endif // HAVE_CUDA

#endif // FTL_CUDA_COMMON_HPP_
