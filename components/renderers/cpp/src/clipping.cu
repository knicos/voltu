#include <ftl/cuda/points.hpp>

#define T_PER_BLOCK 8

__device__ bool isClipped(const float4 &p, const ftl::cuda::ClipSpace &clip) {
	const float3 tp = clip.origin * make_float3(p);
	return fabs(tp.x) > clip.size.x || fabs(tp.y) > clip.size.y || fabs(tp.z) > clip.size.z;
}

__global__ void clipping_kernel(ftl::cuda::TextureObject<float> depth, ftl::rgbd::Camera camera, ftl::cuda::ClipSpace clip)
{
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < depth.width() && y < depth.height()) {
		float d = depth(x,y);
		float4 p = make_float4(camera.screenToCam(x,y,d), 0.0f);

		if (isClipped(p, clip)) {
			depth(x,y) = MINF;
		}
	}
}

void ftl::cuda::clipping(ftl::cuda::TextureObject<float> &depth,
		const ftl::rgbd::Camera &camera,
		const ClipSpace &clip, cudaStream_t stream) {

	const dim3 gridSize((depth.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	clipping_kernel<<<gridSize, blockSize, 0, stream>>>(depth, camera, clip);
	cudaSafeCall( cudaGetLastError() );
}