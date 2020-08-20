#include <ftl/cuda/points.hpp>

#define T_PER_BLOCK 8

__device__ bool isClipped(const float4 &p, const ftl::cuda::ClipSpace &clip) {
	const float3 tp = clip.origin * make_float3(p);
	return fabs(tp.x) > clip.size.x/2.0f || fabs(tp.y) > clip.size.y/2.0f || fabs(tp.z) > clip.size.z/2.0f;
}

__global__ void clipping_kernel(ftl::cuda::TextureObject<float> depth, ftl::rgbd::Camera camera, ftl::cuda::ClipSpace clip)
{
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < depth.width() && y < depth.height()) {
		float d = depth(x,y);
		float4 p = make_float4(camera.screenToCam(x,y,d), 0.0f);

		if (isClipped(p, clip)) {
			depth(x,y) = 0.0f;
		}
	}
}

__global__ void clipping_kernel(ftl::cuda::TextureObject<float> depth, ftl::cuda::TextureObject<uchar4> colour, ftl::rgbd::Camera camera, ftl::cuda::ClipSpace clip)
{
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	const float sx = float(x) / float(colour.width()) * float(depth.width());
	const float sy = float(y) / float(colour.height()) * float(depth.height());

	if (sx >= 0.0f && sx < depth.width() && sy < depth.height() && sy >= 0.0f) {
		float d = depth(sx,sy);
		float4 p = make_float4(camera.screenToCam(sx,sy,d), 0.0f);

		if (d <= camera.minDepth || d >= camera.maxDepth || isClipped(p, clip)) {
			depth(sx,sy) = 0.0f;
			colour(x,y) = make_uchar4(0,0,0,0);
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

void ftl::cuda::clipping(ftl::cuda::TextureObject<float> &depth,
		ftl::cuda::TextureObject<uchar4> &colour,
		const ftl::rgbd::Camera &camera,
		const ClipSpace &clip, cudaStream_t stream) {

	const dim3 gridSize((colour.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (colour.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	clipping_kernel<<<gridSize, blockSize, 0, stream>>>(depth, colour, camera, clip);
	cudaSafeCall( cudaGetLastError() );
}