#include <ftl/cuda/points.hpp>

#define T_PER_BLOCK 8

template <int RADIUS>
__global__ void point_cloud_kernel(ftl::cuda::TextureObject<float4> output, ftl::cuda::TextureObject<float> depth, ftl::rgbd::Camera params, float4x4 pose)
{
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < params.width && y < params.height) {
		output(x,y) = make_float4(MINF, MINF, MINF, MINF);

		const float d = depth.tex2D((int)x, (int)y);
		float p = d;

		if (d >= params.minDepth && d <= params.maxDepth) {
			/* Orts-Escolano S. et al. 2016. Holoportation: Virtual 3D teleportation in real-time. */
			// Is there a discontinuity nearby?
			for (int u=-RADIUS; u<=RADIUS; ++u) {
				for (int v=-RADIUS; v<=RADIUS; ++v) {
					// If yes, the flag using w = -1
					if (fabs(depth.tex2D((int)x+u, (int)y+v) - d) > 0.1f) p = -1.0f;
				}
			}

			output(x,y) = make_float4(pose * params.screenToCam(x, y, d), p);
		}
	}
}

template <>
__global__ void point_cloud_kernel<0>(ftl::cuda::TextureObject<float4> output, ftl::cuda::TextureObject<float> depth, ftl::rgbd::Camera params, float4x4 pose)
{
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < params.width && y < params.height) {
		output(x,y) = make_float4(MINF, MINF, MINF, MINF);

		float d = depth.tex2D((int)x, (int)y);

		if (d >= params.minDepth && d <= params.maxDepth) {
			output(x,y) = make_float4(pose * params.screenToCam(x, y, d), d);
		}
	}
}

void ftl::cuda::point_cloud(ftl::cuda::TextureObject<float4> &output, ftl::cuda::TextureObject<float> &depth, const ftl::rgbd::Camera &params, const float4x4 &pose, uint discon, cudaStream_t stream) {
	const dim3 gridSize((params.width + T_PER_BLOCK - 1)/T_PER_BLOCK, (params.height + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	switch (discon) {
	case 4 :	point_cloud_kernel<4><<<gridSize, blockSize, 0, stream>>>(output, depth, params, pose); break;
	case 3 :	point_cloud_kernel<3><<<gridSize, blockSize, 0, stream>>>(output, depth, params, pose); break;
	case 2 :	point_cloud_kernel<2><<<gridSize, blockSize, 0, stream>>>(output, depth, params, pose); break;
	case 1 :	point_cloud_kernel<1><<<gridSize, blockSize, 0, stream>>>(output, depth, params, pose); break;
	default:	point_cloud_kernel<0><<<gridSize, blockSize, 0, stream>>>(output, depth, params, pose);
	}
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
