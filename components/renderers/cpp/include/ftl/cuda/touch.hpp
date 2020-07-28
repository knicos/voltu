#ifndef _FTL_CUDA_TOUCH_HPP_
#define _FTL_CUDA_TOUCH_HPP_

#include <ftl/cuda_common.hpp>

namespace ftl {
namespace cuda {

struct Collision {
	uint screen;
	float depth;

	__host__ __device__ inline int x() const  { return (screen >> 12) & 0x3FF; }
	__host__ __device__ inline int y() const  { return screen & 0x3FF; }
	__host__ __device__ inline float strength() const { return float(screen >> 24) / 32.0f; }
};

void touch_merge(
		ftl::cuda::TextureObject<float> &depth_in,
		ftl::cuda::TextureObject<float> &depth_out,
        Collision *collisions, int max_collisions,
		float dist,
		cudaStream_t stream);

}
}

#endif
