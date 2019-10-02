#include <ftl/cuda/normals.hpp>
#include <ftl/cuda/weighting.hpp>

#define T_PER_BLOCK 16
#define MINF __int_as_float(0xff800000)

__global__ void computeNormals_kernel(ftl::cuda::TextureObject<float4> output,
        ftl::cuda::TextureObject<float4> input) {
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if(x >= input.width() || y >= input.height()) return;

	output(x,y) = make_float4(0, 0, 0, 0);

	if(x > 0 && x < input.width()-1 && y > 0 && y < input.height()-1) {
		const float3 CC = make_float3(input.tex2D((int)x+0, (int)y+0)); //[(y+0)*width+(x+0)];
		const float3 PC = make_float3(input.tex2D((int)x+0, (int)y+1)); //[(y+1)*width+(x+0)];
		const float3 CP = make_float3(input.tex2D((int)x+1, (int)y+0)); //[(y+0)*width+(x+1)];
		const float3 MC = make_float3(input.tex2D((int)x+0, (int)y-1)); //[(y-1)*width+(x+0)];
		const float3 CM = make_float3(input.tex2D((int)x-1, (int)y+0)); //[(y+0)*width+(x-1)];

		//if(CC.x != MINF && PC.x != MINF && CP.x != MINF && MC.x != MINF && CM.x != MINF) {
			const float3 n = cross(PC-MC, CP-CM);
			const float  l = length(n);

			if(l > 0.0f) {
				output(x,y) = make_float4(n/-l, 1.0f);
			}
		//}
	}
}

template <int RADIUS>
__global__ void smooth_normals_kernel(ftl::cuda::TextureObject<float4> norms,
        ftl::cuda::TextureObject<float4> output,
        ftl::cuda::TextureObject<float4> points, float smoothing) {
    const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

    if(x >= points.width() || y >= points.height()) return;

    const float3 p0 = make_float3(points.tex2D((int)x,(int)y));
    float3 nsum = make_float3(0.0f);
    float contrib = 0.0f;

    if (p0.x == MINF) return;

    for (int v=-RADIUS; v<=RADIUS; ++v) {
        for (int u=-RADIUS; u<=RADIUS; ++u) {
            const float3 p = make_float3(points.tex2D((int)x+u,(int)y+v));
            if (p.x == MINF) continue;
            const float s = ftl::cuda::spatialWeighting(p0, p, smoothing);

            if (s > 0.0f) {
                const float4 n = norms.tex2D((int)x+u,(int)y+v);
                if (n.w > 0.0f) {
                    nsum += make_float3(n) * s;
                    contrib += s;
                }
            }
        }
    }

    // FIXME: USE A DIFFERENT OUTPUT BUFFER
    //__syncthreads();
    output(x,y) = (contrib > 0.0f) ? make_float4(nsum / contrib, 1.0f) : make_float4(0.0f);
}

void ftl::cuda::normals(ftl::cuda::TextureObject<float4> &output,
        ftl::cuda::TextureObject<float4> &temp,
        ftl::cuda::TextureObject<float4> &input, cudaStream_t stream) {
	const dim3 gridSize((input.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (input.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	computeNormals_kernel<<<gridSize, blockSize, 0, stream>>>(temp, input);
    cudaSafeCall( cudaGetLastError() );

    smooth_normals_kernel<1><<<gridSize, blockSize, 0, stream>>>(temp, output, input, 0.04f);
    cudaSafeCall( cudaGetLastError() );

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
#endif
}

//==============================================================================

__global__ void vis_normals_kernel(ftl::cuda::TextureObject<float4> norm,
        ftl::cuda::TextureObject<float> output,
        ftl::rgbd::Camera camera, float4x4 pose) {
    const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

    if(x >= norm.width() || y >= norm.height()) return;

    output(x,y) = 0.0f;
    float3 ray = make_float3(0.0f, 0.0f, 1.0f); //pose * camera.screenToCam(x,y,1.0f);
    ray = ray / length(ray);
    float3 n = make_float3(norm.tex2D((int)x,(int)y));
    float l = length(n);
    if (l == 0) return;
    n /= l;

    output(x,y) = (1.0f + dot(ray, n))*3.5f;  // FIXME: Do not hard code these value scalings
}

void ftl::cuda::normal_visualise(ftl::cuda::TextureObject<float4> &norm,
        ftl::cuda::TextureObject<float> &output,
        const ftl::rgbd::Camera &camera, const float4x4 &pose,
        cudaStream_t stream) {

    const dim3 gridSize((norm.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (norm.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    vis_normals_kernel<<<gridSize, blockSize, 0, stream>>>(norm, output, camera, pose);

    cudaSafeCall( cudaGetLastError() );
#ifdef _DEBUG
    cudaSafeCall(cudaDeviceSynchronize());
    //cutilCheckMsg(__FUNCTION__);
#endif
}
