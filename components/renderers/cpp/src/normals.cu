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

		if(CC.x != MINF && PC.x != MINF && CP.x != MINF && MC.x != MINF && CM.x != MINF) {
			const float3 n = cross(PC-MC, CP-CM);
			const float  l = length(n);

			if(l > 0.0f) {
				output(x,y) = make_float4(n/-l, 1.0f);
			}
		}
	}
}

__device__ inline bool isValid(const ftl::rgbd::Camera &camera, const float3 &d) {
	return d.z >= camera.minDepth && d.z <= camera.maxDepth;
}

__global__ void computeNormals_kernel(ftl::cuda::TextureObject<float4> output,
		ftl::cuda::TextureObject<int> input, ftl::rgbd::Camera camera, float3x3 pose) {
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if(x >= input.width() || y >= input.height()) return;

	output(x,y) = make_float4(0, 0, 0, 0);

	if(x > 0 && x < input.width()-1 && y > 0 && y < input.height()-1) {
		const float3 CC = camera.screenToCam(x+0, y+0, (float)input.tex2D((int)x+0, (int)y+0) / 100000.0f);
		const float3 PC = camera.screenToCam(x+0, y+1, (float)input.tex2D((int)x+0, (int)y+1) / 100000.0f);
		const float3 CP = camera.screenToCam(x+1, y+0, (float)input.tex2D((int)x+1, (int)y+0) / 100000.0f);
		const float3 MC = camera.screenToCam(x+0, y-1, (float)input.tex2D((int)x+0, (int)y-1) / 100000.0f);
		const float3 CM = camera.screenToCam(x-1, y+0, (float)input.tex2D((int)x-1, (int)y+0) / 100000.0f);

		//if(CC.z <  && PC.x != MINF && CP.x != MINF && MC.x != MINF && CM.x != MINF) {
		if (isValid(camera,CC) && isValid(camera,PC) && isValid(camera,CP) && isValid(camera,MC) && isValid(camera,CM)) {
			const float3 n = cross(PC-MC, CP-CM);
			const float  l = length(n);

			if(l > 0.0f) {
				output(x,y) = make_float4((n/-l), 1.0f);
			}
		}
	}
}

__global__ void computeNormals_kernel(ftl::cuda::TextureObject<float4> output,
        ftl::cuda::TextureObject<float> input, ftl::rgbd::Camera camera) {
    const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

    if(x >= input.width() || y >= input.height()) return;

    output(x,y) = make_float4(0, 0, 0, 0);

    if(x > 0 && x < input.width()-1 && y > 0 && y < input.height()-1) {
        const float3 CC = camera.screenToCam(x+0, y+0, input.tex2D((int)x+0, (int)y+0));
        const float3 PC = camera.screenToCam(x+0, y+1, input.tex2D((int)x+0, (int)y+1));
        const float3 CP = camera.screenToCam(x+1, y+0, input.tex2D((int)x+1, (int)y+0));
        const float3 MC = camera.screenToCam(x+0, y-1, input.tex2D((int)x+0, (int)y-1));
        const float3 CM = camera.screenToCam(x-1, y+0, input.tex2D((int)x-1, (int)y+0));

        //if(CC.z <  && PC.x != MINF && CP.x != MINF && MC.x != MINF && CM.x != MINF) {
        if (isValid(camera,CC) && isValid(camera,PC) && isValid(camera,CP) && isValid(camera,MC) && isValid(camera,CM)) {
            const float3 n = cross(PC-MC, CP-CM);
            const float  l = length(n);

            if(l > 0.0f) {
                output(x,y) = make_float4((n/-l), 1.0f);
            }
        }
    }
}

template <int RADIUS>
__global__ void smooth_normals_kernel(ftl::cuda::TextureObject<float4> norms,
        ftl::cuda::TextureObject<float4> output,
        ftl::cuda::TextureObject<float4> points,
        ftl::rgbd::Camera camera, float3x3 pose, float smoothing) {
    const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

    if(x >= points.width() || y >= points.height()) return;

    const float3 p0 = make_float3(points.tex2D((int)x,(int)y));
    float3 nsum = make_float3(0.0f);
    float contrib = 0.0f;

    output(x,y) = make_float4(0.0f,0.0f,0.0f,0.0f);

    if (p0.x == MINF) return;

    for (int v=-RADIUS; v<=RADIUS; ++v) {
        for (int u=-RADIUS; u<=RADIUS; ++u) {
            const float3 p = make_float3(points.tex2D((int)x+u,(int)y+v));
            if (p.x == MINF) continue;
            const float s = ftl::cuda::spatialWeighting(p0, p, smoothing);
            //const float s = 1.0f;

            if (s > 0.0f) {
                const float4 n = norms.tex2D((int)x+u,(int)y+v);
                if (n.w > 0.0f) {
                    nsum += make_float3(n) * s;
                    contrib += s;
                }
            }
        }
    }

    // Compute dot product of normal with camera to obtain measure of how
    // well this point faces the source camera, a measure of confidence
    float3 ray = pose * camera.screenToCam(x, y, 1.0f);
    ray = ray / length(ray);
    nsum /= contrib;
    nsum /= length(nsum);

    output(x,y) = (contrib > 0.0f) ? make_float4(nsum, dot(nsum, ray)) : make_float4(0.0f);
}

template <int RADIUS>
__global__ void smooth_normals_kernel(ftl::cuda::TextureObject<float4> norms,
        ftl::cuda::TextureObject<float4> output,
        ftl::cuda::TextureObject<int> depth,
        ftl::rgbd::Camera camera, float3x3 pose, float smoothing) {
    const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

    if(x >= depth.width() || y >= depth.height()) return;

    const float3 p0 = camera.screenToCam(x,y, (float)depth.tex2D((int)x,(int)y) / 100000.0f);
    float3 nsum = make_float3(0.0f);
    float contrib = 0.0f;

    output(x,y) = make_float4(0.0f,0.0f,0.0f,0.0f);

    if (p0.z < camera.minDepth || p0.z > camera.maxDepth) return;

    for (int v=-RADIUS; v<=RADIUS; ++v) {
        for (int u=-RADIUS; u<=RADIUS; ++u) {
            const float3 p = camera.screenToCam(x+u,y+v, (float)depth.tex2D((int)x+u,(int)y+v) / 100000.0f);
            if (p.z < camera.minDepth || p.z > camera.maxDepth) continue;
            const float s = ftl::cuda::spatialWeighting(p0, p, smoothing);
            //const float s = 1.0f;

            //if (s > 0.0f) {
                const float4 n = norms.tex2D((int)x+u,(int)y+v);
                if (n.w > 0.0f) {
                    nsum += make_float3(n) * s;
                    contrib += s;
                }
            //}
        }
    }

    // Compute dot product of normal with camera to obtain measure of how
    // well this point faces the source camera, a measure of confidence
    float3 ray = camera.screenToCam(x, y, 1.0f);
    ray = ray / length(ray);
    nsum /= contrib;
    nsum /= length(nsum);

    output(x,y) = (contrib > 0.0f) ? make_float4(pose*nsum, 1.0f) : make_float4(0.0f);
}

template <>
__global__ void smooth_normals_kernel<0>(ftl::cuda::TextureObject<float4> norms,
        ftl::cuda::TextureObject<float4> output,
        ftl::cuda::TextureObject<int> depth,
        ftl::rgbd::Camera camera, float3x3 pose, float smoothing) {
    const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

    if(x >= depth.width() || y >= depth.height()) return;

    output(x,y) = make_float4(0.0f,0.0f,0.0f,0.0f);

    const float3 p0 = camera.screenToCam(x,y, (float)depth.tex2D((int)x,(int)y) / 1000.0f);

    if (p0.z < camera.minDepth || p0.z > camera.maxDepth) return;

    // Compute dot product of normal with camera to obtain measure of how
    // well this point faces the source camera, a measure of confidence
    //float3 ray = camera.screenToCam(x, y, 1.0f);
    //ray = ray / length(ray);
    //nsum /= contrib;
    //nsum /= length(nsum);

    const float4 n = norms.tex2D((int)x,(int)y);
    output(x,y) = n;
}

template <int RADIUS>
__global__ void smooth_normals_kernel(ftl::cuda::TextureObject<float4> norms,
        ftl::cuda::TextureObject<float4> output,
        ftl::cuda::TextureObject<float> depth,
        ftl::rgbd::Camera camera, float3x3 pose, float smoothing) {
    const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

    if(x >= depth.width() || y >= depth.height()) return;

    const float3 p0 = camera.screenToCam(x,y, depth.tex2D((int)x,(int)y));
    float3 nsum = make_float3(0.0f);
    float contrib = 0.0f;

    output(x,y) = make_float4(0.0f,0.0f,0.0f,0.0f);

    if (p0.z < camera.minDepth || p0.z > camera.maxDepth) return;

    for (int v=-RADIUS; v<=RADIUS; ++v) {
        for (int u=-RADIUS; u<=RADIUS; ++u) {
            const float3 p = camera.screenToCam(x+u,y+v, depth.tex2D((int)x+u,(int)y+v));
            if (p.z < camera.minDepth || p.z > camera.maxDepth) continue;
            const float s = ftl::cuda::spatialWeighting(p0, p, smoothing);
            //const float s = 1.0f;

            //if (s > 0.0f) {
                const float4 n = norms.tex2D((int)x+u,(int)y+v);
                if (n.w > 0.0f) {
                    nsum += make_float3(n) * s;
                    contrib += s;
                }
            //}
        }
    }

    // Compute dot product of normal with camera to obtain measure of how
    // well this point faces the source camera, a measure of confidence
    float3 ray = camera.screenToCam(x, y, 1.0f);
    ray = ray / length(ray);
    nsum /= contrib;
    nsum /= length(nsum);

    output(x,y) = (contrib > 0.0f) ? make_float4(pose*nsum, 1.0f) : make_float4(0.0f);
}

void ftl::cuda::normals(ftl::cuda::TextureObject<float4> &output,
        ftl::cuda::TextureObject<float4> &temp,
		ftl::cuda::TextureObject<float4> &input,
		int radius,
		float smoothing,
        const ftl::rgbd::Camera &camera,
        const float3x3 &pose,cudaStream_t stream) {
	const dim3 gridSize((input.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (input.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	computeNormals_kernel<<<gridSize, blockSize, 0, stream>>>(temp, input);
    cudaSafeCall( cudaGetLastError() );

	switch (radius) {
	case 9: smooth_normals_kernel<9><<<gridSize, blockSize, 0, stream>>>(temp, output, input, camera, pose, smoothing); break;
	case 7: smooth_normals_kernel<7><<<gridSize, blockSize, 0, stream>>>(temp, output, input, camera, pose, smoothing); break;
	case 5: smooth_normals_kernel<5><<<gridSize, blockSize, 0, stream>>>(temp, output, input, camera, pose, smoothing); break;
    case 3: smooth_normals_kernel<3><<<gridSize, blockSize, 0, stream>>>(temp, output, input, camera, pose, smoothing); break;
    case 2: smooth_normals_kernel<2><<<gridSize, blockSize, 0, stream>>>(temp, output, input, camera, pose, smoothing); break;
    case 1: smooth_normals_kernel<1><<<gridSize, blockSize, 0, stream>>>(temp, output, input, camera, pose, smoothing); break;
    case 0: smooth_normals_kernel<0><<<gridSize, blockSize, 0, stream>>>(temp, output, input, camera, pose, smoothing); break;
	}
    cudaSafeCall( cudaGetLastError() );

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
#endif
}

void ftl::cuda::normals(ftl::cuda::TextureObject<float4> &output,
		ftl::cuda::TextureObject<float4> &temp,
		ftl::cuda::TextureObject<int> &input,
		int radius,
		float smoothing,
		const ftl::rgbd::Camera &camera,
		const float3x3 &pose_inv, const float3x3 &pose,cudaStream_t stream) {
	const dim3 gridSize((input.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (input.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	computeNormals_kernel<<<gridSize, blockSize, 0, stream>>>(temp, input, camera, pose);
	cudaSafeCall( cudaGetLastError() );

	switch (radius) {
	case 7: smooth_normals_kernel<7><<<gridSize, blockSize, 0, stream>>>(temp, output, input, camera, pose, smoothing);
	case 5: smooth_normals_kernel<5><<<gridSize, blockSize, 0, stream>>>(temp, output, input, camera, pose, smoothing);
    case 3: smooth_normals_kernel<3><<<gridSize, blockSize, 0, stream>>>(temp, output, input, camera, pose, smoothing);
    case 2: smooth_normals_kernel<2><<<gridSize, blockSize, 0, stream>>>(temp, output, input, camera, pose, smoothing);
    case 1: smooth_normals_kernel<1><<<gridSize, blockSize, 0, stream>>>(temp, output, input, camera, pose, smoothing);
	}
	cudaSafeCall( cudaGetLastError() );

	#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
	#endif
}

void ftl::cuda::normals(ftl::cuda::TextureObject<float4> &output,
		ftl::cuda::TextureObject<float4> &temp,
		ftl::cuda::TextureObject<float> &input,
		int radius,
		float smoothing,
		const ftl::rgbd::Camera &camera,
		const float3x3 &pose_inv, const float3x3 &pose,cudaStream_t stream) {
	const dim3 gridSize((input.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (input.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	computeNormals_kernel<<<gridSize, blockSize, 0, stream>>>(temp, input, camera);
	cudaSafeCall( cudaGetLastError() );

	switch (radius) {
	case 7: smooth_normals_kernel<7><<<gridSize, blockSize, 0, stream>>>(temp, output, input, camera, pose, smoothing);
	case 5: smooth_normals_kernel<5><<<gridSize, blockSize, 0, stream>>>(temp, output, input, camera, pose, smoothing);
	case 3: smooth_normals_kernel<3><<<gridSize, blockSize, 0, stream>>>(temp, output, input, camera, pose, smoothing);
	case 2: smooth_normals_kernel<2><<<gridSize, blockSize, 0, stream>>>(temp, output, input, camera, pose, smoothing);
	case 1: smooth_normals_kernel<1><<<gridSize, blockSize, 0, stream>>>(temp, output, input, camera, pose, smoothing);
	}
	cudaSafeCall( cudaGetLastError() );

	#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
	#endif
}

void ftl::cuda::normals(ftl::cuda::TextureObject<float4> &output,
        ftl::cuda::TextureObject<float> &input,
        const ftl::rgbd::Camera &camera,
        cudaStream_t stream) {
    const dim3 gridSize((input.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (input.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    computeNormals_kernel<<<gridSize, blockSize, 0, stream>>>(output, input, camera);
    cudaSafeCall( cudaGetLastError() );

    #ifdef _DEBUG
    cudaSafeCall(cudaDeviceSynchronize());
    //cutilCheckMsg(__FUNCTION__);
    #endif
}

// =============================================================================

__global__ void computeNormals_dot_kernel(ftl::cuda::TextureObject<float> output,
		ftl::cuda::TextureObject<float> input, ftl::rgbd::Camera camera) {
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if(x >= input.width() || y >= input.height()) return;

	float out = 0.0f;

	if(x > 0 && x < input.width()-1 && y > 0 && y < input.height()-1) {
		float d = input.tex2D((int)x+0, (int)y+0);
		const float3 CC = camera.screenToCam(x+0, y+0, d);
		const float3 PC = camera.screenToCam(x+0, y+1, input.tex2D((int)x+0, (int)y+1));
		const float3 CP = camera.screenToCam(x+1, y+0, input.tex2D((int)x+1, (int)y+0));
		const float3 MC = camera.screenToCam(x+0, y-1, input.tex2D((int)x+0, (int)y-1));
		const float3 CM = camera.screenToCam(x-1, y+0, input.tex2D((int)x-1, (int)y+0));

		//if(CC.z <  && PC.x != MINF && CP.x != MINF && MC.x != MINF && CM.x != MINF) {
		if (isValid(camera,CC) && isValid(camera,PC) && isValid(camera,CP) && isValid(camera,MC) && isValid(camera,CM)) {
			float3 n = cross(PC-MC, CP-CM);
			const float  l = length(n);

			if(l > 0.0f) {
				n = n / -l;
				float3 ray = camera.screenToCam(x, y, d);
				ray = ray / length(ray);
				out = dot(ray,n);
			}
		}
	}

	output(x,y) = out;
}

void ftl::cuda::normals_dot(ftl::cuda::TextureObject<float> &output,
		ftl::cuda::TextureObject<float> &input,
		const ftl::rgbd::Camera &camera,
		cudaStream_t stream) {
	const dim3 gridSize((input.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (input.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	computeNormals_dot_kernel<<<gridSize, blockSize, 0, stream>>>(output, input, camera);
	cudaSafeCall( cudaGetLastError() );

	#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
	#endif
}

//==============================================================================

__global__ void vis_normals_kernel(ftl::cuda::TextureObject<float4> norm,
        ftl::cuda::TextureObject<uchar4> output,
        float3 direction, uchar4 diffuse, uchar4 ambient) {
    const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

    if(x >= norm.width() || y >= norm.height()) return;

    output(x,y) = make_uchar4(0,0,0,0);
    float3 ray = direction;
    ray = ray / length(ray);
    float3 n = make_float3(norm.tex2D((int)x,(int)y));
    float l = length(n);
    if (l == 0) return;
    n /= l;

    const float d = max(dot(ray, n), 0.0f);
    output(x,y) = make_uchar4(
		min(255.0f, diffuse.x*d + ambient.x),
		min(255.0f, diffuse.y*d + ambient.y),
		min(255.0f, diffuse.z*d + ambient.z), 255);
}

void ftl::cuda::normal_visualise(ftl::cuda::TextureObject<float4> &norm,
        ftl::cuda::TextureObject<uchar4> &output,
        const float3 &light, const uchar4 &diffuse, const uchar4 &ambient,
        cudaStream_t stream) {

    const dim3 gridSize((norm.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (norm.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    vis_normals_kernel<<<gridSize, blockSize, 0, stream>>>(norm, output, light, diffuse, ambient);

    cudaSafeCall( cudaGetLastError() );
#ifdef _DEBUG
    cudaSafeCall(cudaDeviceSynchronize());
    //cutilCheckMsg(__FUNCTION__);
#endif
}

//==============================================================================

__global__ void cool_blue_kernel(ftl::cuda::TextureObject<float4> norm,
        ftl::cuda::TextureObject<uchar4> output,
        uchar4 colouring, float3x3 pose) {
    const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

    if(x >= norm.width() || y >= norm.height()) return;

    //output(x,y) = make_uchar4(0,0,0,0);
    float3 ray = pose * make_float3(0.0f,0.0f,1.0f);
    ray = ray / length(ray);
    float3 n = make_float3(norm.tex2D((int)x,(int)y));
    float l = length(n);
    if (l == 0) return;
    n /= l;

    const float d = 1.0f - max(dot(ray, n), 0.0f);
    uchar4 original = output(x,y); //.tex2D(x,y);

    output(x,y) = make_uchar4(
        min(255.0f, colouring.x*d + original.x),
        min(255.0f, colouring.y*d + original.y),
        min(255.0f, colouring.z*d + original.z), 255);
}

void ftl::cuda::cool_blue(ftl::cuda::TextureObject<float4> &norm,
        ftl::cuda::TextureObject<uchar4> &output,
        const uchar4 &colouring, const float3x3 &pose,
        cudaStream_t stream) {

    const dim3 gridSize((norm.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (norm.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    cool_blue_kernel<<<gridSize, blockSize, 0, stream>>>(norm, output, colouring, pose);

    cudaSafeCall( cudaGetLastError() );
    #ifdef _DEBUG
    cudaSafeCall(cudaDeviceSynchronize());
    //cutilCheckMsg(__FUNCTION__);
    #endif
}

//==============================================================================

__global__ void filter_normals_kernel(ftl::cuda::TextureObject<float4> norm,
        ftl::cuda::TextureObject<float4> output,
        ftl::rgbd::Camera camera, float4x4 pose, float thresh) {
    const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

    if(x >= norm.width() || y >= norm.height()) return;

    float3 ray = pose.getFloat3x3() * camera.screenToCam(x,y,1.0f);
    ray = ray / length(ray);
    float3 n = make_float3(norm.tex2D((int)x,(int)y));
    float l = length(n);
    if (l == 0) {
        output(x,y) = make_float4(MINF);
        return;
    }
    n /= l;

    const float d = dot(ray, n);
    if (d <= thresh) {
        output(x,y) = make_float4(MINF);
    }
}

void ftl::cuda::normal_filter(ftl::cuda::TextureObject<float4> &norm,
        ftl::cuda::TextureObject<float4> &output,
        const ftl::rgbd::Camera &camera, const float4x4 &pose,
        float thresh,
        cudaStream_t stream) {

    const dim3 gridSize((norm.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (norm.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    filter_normals_kernel<<<gridSize, blockSize, 0, stream>>>(norm, output, camera, pose, thresh);

    cudaSafeCall( cudaGetLastError() );
    #ifdef _DEBUG
    cudaSafeCall(cudaDeviceSynchronize());
    //cutilCheckMsg(__FUNCTION__);
    #endif
}

//==============================================================================

__global__ void transform_normals_kernel(ftl::cuda::TextureObject<float4> norm,
        float3x3 pose) {
    const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

    if(x >= norm.width() || y >= norm.height()) return;

    float3 normal = pose * make_float3(norm.tex2D((int)x,(int)y));
    normal /= length(normal);
    norm(x,y) = make_float4(normal, 0.0f);
}

void ftl::cuda::transform_normals(ftl::cuda::TextureObject<float4> &norm,
        const float3x3 &pose,
        cudaStream_t stream) {

    const dim3 gridSize((norm.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (norm.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    transform_normals_kernel<<<gridSize, blockSize, 0, stream>>>(norm, pose);

    cudaSafeCall( cudaGetLastError() );
    #ifdef _DEBUG
    cudaSafeCall(cudaDeviceSynchronize());
    //cutilCheckMsg(__FUNCTION__);
    #endif
}
