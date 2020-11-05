#include <ftl/operators/cuda/mls_cuda.hpp>
#include <ftl/cuda/weighting.hpp>

// ===== MLS Smooth ============================================================

/*
 * Smooth depth map using Moving Least Squares. This version is for a single
 * depth image and does not use colour.
 */
 template <int RADIUS>
 __global__ void mls_smooth_kernel(
	const half4* __restrict__ normals_in,
	half4* __restrict__ normals_out,	// Can be nullptr
	const float* __restrict__ depth_in,
	float* __restrict__ depth_out,		// Can be nullptr
	int npitch,
	int dpitch,
	float smoothing,					// Radius of Gaussian in cm
	ftl::rgbd::Camera camera
) {	 
   
    const int x = blockIdx.x*blockDim.x + threadIdx.x;
    const int y = blockIdx.y*blockDim.y + threadIdx.y;

    if (x < RADIUS || y < RADIUS || x >= camera.width-RADIUS || y >= camera.height-RADIUS) return;

	float3 aX = make_float3(0.0f,0.0f,0.0f);
	float3 nX = make_float3(0.0f,0.0f,0.0f);
    float contrib = 0.0f;

	const float d0 = depth_in[x+y*dpitch];
	if (depth_out) depth_out[x+y*dpitch] = d0;
	if (normals_out) normals_out[x+y*npitch] = normals_in[x+y*npitch];
	if (d0 < camera.minDepth || d0 > camera.maxDepth) return;

	float3 X = camera.screenToCam((int)(x),(int)(y),d0);

    // Neighbourhood
    for (int v=-RADIUS; v<=RADIUS; ++v) {
    for (int u=-RADIUS; u<=RADIUS; ++u) {
		const float d = depth_in[x+u+(y+v)*dpitch];
		if (d < camera.minDepth || d > camera.maxDepth) continue;

		// Point and normal of neighbour
		const float3 Xi = camera.screenToCam(x+u, y+v, d);
		const float3 Ni = make_float3(normals_in[x+u+(y+v)*npitch]);

		// Gauss approx weighting function using point distance
		const float w = (Ni.x+Ni.y+Ni.z == 0.0f) ? 0.0f : ftl::cuda::spatialWeighting(X,Xi,smoothing);

		aX += Xi*w;
		nX += Ni*w;
		contrib += w;
    }
	}
	
	if (contrib > 0.0f) {
		nX /= contrib;  // Weighted average normal
		aX /= contrib;  // Weighted average point (centroid)

		// Signed-Distance Field function
		float fX = nX.x * (X.x - aX.x) + nX.y * (X.y - aX.y) + nX.z * (X.z - aX.z);

		// Calculate new point using SDF function to adjust depth (and position)
		X = X - nX * fX;
	
		if (depth_out) depth_out[x+y*dpitch] = X.z;
		if (normals_out) normals_out[x+y*npitch] = make_half4(nX / length(nX), 0.0f);
	}
}

/* One iteration of MLS Smoothing, simple, single image and output depth also. */
void ftl::cuda::mls_smooth(
	const cv::cuda::GpuMat &normals_in,
	cv::cuda::GpuMat &normals_out,
	const cv::cuda::GpuMat &depth_in,
	cv::cuda::GpuMat &depth_out,
	float smoothing,
	int radius,
	const ftl::rgbd::Camera &camera,
	cudaStream_t stream
) {
	static constexpr int THREADS_X = 8;
	static constexpr int THREADS_Y = 8;

	const dim3 gridSize((depth_in.cols + THREADS_X - 1)/THREADS_X, (depth_in.rows + THREADS_Y - 1)/THREADS_Y);
	const dim3 blockSize(THREADS_X, THREADS_Y);

	normals_out.create(normals_in.size(), CV_16FC4);
	depth_out.create(depth_in.size(), CV_32F);

	switch (radius) {
		case 5: mls_smooth_kernel<5><<<gridSize, blockSize, 0, stream>>>(normals_in.ptr<half4>(), normals_out.ptr<half4>(), depth_in.ptr<float>(), depth_out.ptr<float>(), normals_in.step1()/4, depth_in.step1(), smoothing, camera); break;
		case 4: mls_smooth_kernel<4><<<gridSize, blockSize, 0, stream>>>(normals_in.ptr<half4>(), normals_out.ptr<half4>(), depth_in.ptr<float>(), depth_out.ptr<float>(), normals_in.step1()/4, depth_in.step1(), smoothing, camera); break;
		case 3: mls_smooth_kernel<3><<<gridSize, blockSize, 0, stream>>>(normals_in.ptr<half4>(), normals_out.ptr<half4>(), depth_in.ptr<float>(), depth_out.ptr<float>(), normals_in.step1()/4, depth_in.step1(), smoothing, camera); break;
		case 2: mls_smooth_kernel<2><<<gridSize, blockSize, 0, stream>>>(normals_in.ptr<half4>(), normals_out.ptr<half4>(), depth_in.ptr<float>(), depth_out.ptr<float>(), normals_in.step1()/4, depth_in.step1(), smoothing, camera); break;
		case 1: mls_smooth_kernel<1><<<gridSize, blockSize, 0, stream>>>(normals_in.ptr<half4>(), normals_out.ptr<half4>(), depth_in.ptr<float>(), depth_out.ptr<float>(), normals_in.step1()/4, depth_in.step1(), smoothing, camera); break;
	}
	cudaSafeCall( cudaGetLastError() );
}

/* One iteration of MLS Smoothing, simple, single image, normals only */
void ftl::cuda::mls_smooth(
	const cv::cuda::GpuMat &normals_in,
	cv::cuda::GpuMat &normals_out,
	const cv::cuda::GpuMat &depth_in,
	float smoothing,
	int radius,
	const ftl::rgbd::Camera &camera,
	cudaStream_t stream
) {
	static constexpr int THREADS_X = 8;
	static constexpr int THREADS_Y = 8;

	const dim3 gridSize((depth_in.cols + THREADS_X - 1)/THREADS_X, (depth_in.rows + THREADS_Y - 1)/THREADS_Y);
	const dim3 blockSize(THREADS_X, THREADS_Y);

	normals_out.create(normals_in.size(), CV_16FC4);

	switch (radius) {
		case 5: mls_smooth_kernel<5><<<gridSize, blockSize, 0, stream>>>(normals_in.ptr<half4>(), normals_out.ptr<half4>(), depth_in.ptr<float>(), nullptr, normals_in.step1()/4, depth_in.step1(), smoothing, camera); break;
		case 4: mls_smooth_kernel<4><<<gridSize, blockSize, 0, stream>>>(normals_in.ptr<half4>(), normals_out.ptr<half4>(), depth_in.ptr<float>(), nullptr, normals_in.step1()/4, depth_in.step1(), smoothing, camera); break;
		case 3: mls_smooth_kernel<3><<<gridSize, blockSize, 0, stream>>>(normals_in.ptr<half4>(), normals_out.ptr<half4>(), depth_in.ptr<float>(), nullptr, normals_in.step1()/4, depth_in.step1(), smoothing, camera); break;
		case 2: mls_smooth_kernel<2><<<gridSize, blockSize, 0, stream>>>(normals_in.ptr<half4>(), normals_out.ptr<half4>(), depth_in.ptr<float>(), nullptr, normals_in.step1()/4, depth_in.step1(), smoothing, camera); break;
		case 1: mls_smooth_kernel<1><<<gridSize, blockSize, 0, stream>>>(normals_in.ptr<half4>(), normals_out.ptr<half4>(), depth_in.ptr<float>(), nullptr, normals_in.step1()/4, depth_in.step1(), smoothing, camera); break;
	}
	cudaSafeCall( cudaGetLastError() );
}
