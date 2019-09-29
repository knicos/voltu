#include <ftl/cuda/points.hpp>

#define T_PER_BLOCK 8

__global__ void point_cloud_kernel(ftl::cuda::TextureObject<float4> output, ftl::cuda::TextureObject<float> depth, ftl::rgbd::Camera params, float4x4 pose)
{
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < params.width && y < params.height) {
		float d = depth.tex2D((int)x, (int)y);

		output(x,y) = (d >= params.minDepth && d <= params.maxDepth) ?
			make_float4(pose * params.screenToCam(x, y, d), 0.0f) :
			make_float4(MINF, MINF, MINF, MINF);
	}
}

void ftl::cuda::point_cloud(ftl::cuda::TextureObject<float4> &output, ftl::cuda::TextureObject<float> &depth, const ftl::rgbd::Camera &params, const float4x4 &pose, cudaStream_t stream) {
	const dim3 gridSize((params.width + T_PER_BLOCK - 1)/T_PER_BLOCK, (params.height + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	point_cloud_kernel<<<gridSize, blockSize, 0, stream>>>(output, depth, params, pose);
	cudaSafeCall( cudaGetLastError() );

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
#endif
}

//==============================================================================

__device__ bool isClipped(const float4 &p, const ftl::cuda::ClipSpace &clip) {
	const float3 tp = clip.origin * make_float3(p);
	return fabs(tp.x) > clip.size.x || fabs(tp.y) > clip.size.y || fabs(tp.z) > clip.size.z;
}

__global__ void clipping_kernel(ftl::cuda::TextureObject<float4> points, ftl::cuda::ClipSpace clip)
{
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < points.width() && y < points.height()) {
		float4 p = points(x,y);

		if (isClipped(p, clip)) {
			points(x,y) = make_float4(MINF, MINF, MINF, MINF);
		}
	}
}

void ftl::cuda::clipping(ftl::cuda::TextureObject<float4> &points,
		const ClipSpace &clip, cudaStream_t stream) {

	const dim3 gridSize((points.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (points.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	clipping_kernel<<<gridSize, blockSize, 0, stream>>>(points, clip);
	cudaSafeCall( cudaGetLastError() );
}
