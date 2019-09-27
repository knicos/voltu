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

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
#endif
}
