#include "smoothing_cuda.hpp"

#include <ftl/cuda/weighting.hpp>

using ftl::cuda::TextureObject;

#define T_PER_BLOCK 8

// ===== MLS Smooth ============================================================

/*
 * Smooth depth map using Moving Least Squares
 */
 template <int SEARCH_RADIUS>
 __global__ void mls_smooth_kernel(
		TextureObject<float4> normals_in,
		TextureObject<float4> normals_out,
        TextureObject<float> depth_in,        // Virtual depth map
		TextureObject<float> depth_out,   // Accumulated output
		float smoothing,
        ftl::rgbd::Camera camera) {
        
    const int x = blockIdx.x*blockDim.x + threadIdx.x;
    const int y = blockIdx.y*blockDim.y + threadIdx.y;

    if (x < 0 || y < 0 || x >= depth_in.width() || y >= depth_in.height()) return;

	float3 aX = make_float3(0.0f,0.0f,0.0f);
	float3 nX = make_float3(0.0f,0.0f,0.0f);
    float contrib = 0.0f;

	float d0 = depth_in.tex2D(x, y);
	depth_out(x,y) = d0;
	if (d0 < camera.minDepth || d0 > camera.maxDepth) return;
	float3 X = camera.screenToCam((int)(x),(int)(y),d0);

    // Neighbourhood
    for (int v=-SEARCH_RADIUS; v<=SEARCH_RADIUS; ++v) {
    for (int u=-SEARCH_RADIUS; u<=SEARCH_RADIUS; ++u) {
		const float d = depth_in.tex2D(x+u, y+v);
		if (d < camera.minDepth || d > camera.maxDepth) continue;

		// Point and normal of neighbour
		const float3 Xi = camera.screenToCam((int)(x)+u,(int)(y)+v,d);
		const float3 Ni = make_float3(normals_in.tex2D((int)(x)+u, (int)(y)+v));

		// Gauss approx weighting function using point distance
		const float w = ftl::cuda::spatialWeighting(X,Xi,smoothing);

		aX += Xi*w;
		nX += Ni*w;
		contrib += w;
    }
	}
	
	nX /= contrib;  // Weighted average normal
	aX /= contrib;  // Weighted average point (centroid)

	// Signed-Distance Field function
	float fX = nX.x * (X.x - aX.x) + nX.y * (X.y - aX.y) + nX.z * (X.z - aX.z);

	// Calculate new point using SDF function to adjust depth (and position)
	X = X - nX * fX;
	
	//uint2 screen = camera.camToScreen<uint2>(X);

    //if (screen.x < depth_out.width() && screen.y < depth_out.height()) {
    //    depth_out(screen.x,screen.y) = X.z;
	//}
	depth_out(x,y) = X.z;
	normals_out(x,y) = make_float4(nX / length(nX), 0.0f);
}

void ftl::cuda::mls_smooth(
		ftl::cuda::TextureObject<float4> &normals_in,
		ftl::cuda::TextureObject<float4> &normals_out,
		ftl::cuda::TextureObject<float> &depth_in,
		ftl::cuda::TextureObject<float> &depth_out,
		float smoothing,
		int radius,
		const ftl::rgbd::Camera &camera,
		cudaStream_t stream) {

	const dim3 gridSize((depth_out.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth_out.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	switch (radius) {
		case 5: mls_smooth_kernel<5><<<gridSize, blockSize, 0, stream>>>(normals_in, normals_out, depth_in, depth_out, smoothing, camera); break;
		case 4: mls_smooth_kernel<4><<<gridSize, blockSize, 0, stream>>>(normals_in, normals_out, depth_in, depth_out, smoothing, camera); break;
		case 3: mls_smooth_kernel<3><<<gridSize, blockSize, 0, stream>>>(normals_in, normals_out, depth_in, depth_out, smoothing, camera); break;
		case 2: mls_smooth_kernel<2><<<gridSize, blockSize, 0, stream>>>(normals_in, normals_out, depth_in, depth_out, smoothing, camera); break;
		case 1: mls_smooth_kernel<1><<<gridSize, blockSize, 0, stream>>>(normals_in, normals_out, depth_in, depth_out, smoothing, camera); break;
	}
	cudaSafeCall( cudaGetLastError() );


	#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	#endif
}


// ===== Colour MLS Smooth =====================================================

/*
 * Smooth depth map using Moving Least Squares. This version uses colour
 * similarity weights to adjust the spatial smoothing factor. It is naive in
 * that similar colours may exist on both sides of an edge and colours at the
 * level of single pixels can be subject to noise. Colour noise should first
 * be removed from the image.
 */
 template <int SEARCH_RADIUS>
 __global__ void colour_mls_smooth_kernel(
		TextureObject<float4> normals_in,
		TextureObject<float4> normals_out,
        TextureObject<float> depth_in,        // Virtual depth map
		TextureObject<float> depth_out,   // Accumulated output
		TextureObject<uchar4> colour_in,
		float smoothing,
		float colour_smoothing,
        ftl::rgbd::Camera camera) {
        
    const int x = blockIdx.x*blockDim.x + threadIdx.x;
    const int y = blockIdx.y*blockDim.y + threadIdx.y;

    if (x < 0 || y < 0 || x >= depth_in.width() || y >= depth_in.height()) return;

	float3 aX = make_float3(0.0f,0.0f,0.0f);
	float3 nX = make_float3(0.0f,0.0f,0.0f);
    float contrib = 0.0f;

	float d0 = depth_in.tex2D(x, y);
	depth_out(x,y) = d0;
	if (d0 < camera.minDepth || d0 > camera.maxDepth) return;
	float3 X = camera.screenToCam((int)(x),(int)(y),d0);

	uchar4 c0 = colour_in.tex2D(x, y);

    // Neighbourhood
    for (int v=-SEARCH_RADIUS; v<=SEARCH_RADIUS; ++v) {
    for (int u=-SEARCH_RADIUS; u<=SEARCH_RADIUS; ++u) {
		const float d = depth_in.tex2D(x+u, y+v);
		if (d < camera.minDepth || d > camera.maxDepth) continue;

		// Point and normal of neighbour
		const float3 Xi = camera.screenToCam((int)(x)+u,(int)(y)+v,d);
		const float3 Ni = make_float3(normals_in.tex2D((int)(x)+u, (int)(y)+v));

		const uchar4 c = colour_in.tex2D(x+u, y+v);
		const float cw = ftl::cuda::colourWeighting(c0,c,colour_smoothing);

		// Gauss approx weighting function using point distance
		const float w = ftl::cuda::spatialWeighting(X,Xi,smoothing*cw);

		aX += Xi*w;
		nX += Ni*w;
		contrib += w;
    }
	}
	
	nX /= contrib;  // Weighted average normal
	aX /= contrib;  // Weighted average point (centroid)

	// Signed-Distance Field function
	float fX = nX.x * (X.x - aX.x) + nX.y * (X.y - aX.y) + nX.z * (X.z - aX.z);

	// Calculate new point using SDF function to adjust depth (and position)
	X = X - nX * fX;
	
	//uint2 screen = camera.camToScreen<uint2>(X);

    //if (screen.x < depth_out.width() && screen.y < depth_out.height()) {
    //    depth_out(screen.x,screen.y) = X.z;
	//}
	depth_out(x,y) = X.z;
	normals_out(x,y) = make_float4(nX / length(nX), 0.0f);
}

void ftl::cuda::colour_mls_smooth(
		ftl::cuda::TextureObject<float4> &normals_in,
		ftl::cuda::TextureObject<float4> &normals_out,
		ftl::cuda::TextureObject<float> &depth_in,
		ftl::cuda::TextureObject<float> &depth_out,
		ftl::cuda::TextureObject<uchar4> &colour_in,
		float smoothing,
		float colour_smoothing,
		int radius,
		const ftl::rgbd::Camera &camera,
		cudaStream_t stream) {

	const dim3 gridSize((depth_out.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth_out.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	switch (radius) {
		case 5: colour_mls_smooth_kernel<5><<<gridSize, blockSize, 0, stream>>>(normals_in, normals_out, depth_in, depth_out, colour_in, smoothing, colour_smoothing, camera); break;
		case 4: colour_mls_smooth_kernel<4><<<gridSize, blockSize, 0, stream>>>(normals_in, normals_out, depth_in, depth_out, colour_in, smoothing, colour_smoothing, camera); break;
		case 3: colour_mls_smooth_kernel<3><<<gridSize, blockSize, 0, stream>>>(normals_in, normals_out, depth_in, depth_out, colour_in, smoothing, colour_smoothing, camera); break;
		case 2: colour_mls_smooth_kernel<2><<<gridSize, blockSize, 0, stream>>>(normals_in, normals_out, depth_in, depth_out, colour_in, smoothing, colour_smoothing, camera); break;
		case 1: colour_mls_smooth_kernel<1><<<gridSize, blockSize, 0, stream>>>(normals_in, normals_out, depth_in, depth_out, colour_in, smoothing, colour_smoothing, camera); break;
	}
	cudaSafeCall( cudaGetLastError() );


	#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	#endif
}


// ===== Adaptive MLS Smooth =====================================================

/*
 * Smooth depth map using Moving Least Squares. This version uses colour
 * similarity weights to adjust the spatial smoothing factor. It is naive in
 * that similar colours may exist on both sides of an edge and colours at the
 * level of single pixels can be subject to noise. Colour noise should first
 * be removed from the image.
 */
 template <int SEARCH_RADIUS>
 __global__ void adaptive_mls_smooth_kernel(
		TextureObject<float4> normals_in,
		TextureObject<float4> normals_out,
        TextureObject<float> depth_in,        // Virtual depth map
		TextureObject<float> depth_out,   // Accumulated output
		TextureObject<float> smoothing,
        ftl::rgbd::Camera camera) {
        
    const int x = blockIdx.x*blockDim.x + threadIdx.x;
    const int y = blockIdx.y*blockDim.y + threadIdx.y;

    if (x < 0 || y < 0 || x >= depth_in.width() || y >= depth_in.height()) return;

	float3 aX = make_float3(0.0f,0.0f,0.0f);
	float3 nX = make_float3(0.0f,0.0f,0.0f);
    float contrib = 0.0f;

	float d0 = depth_in.tex2D(x, y);
	depth_out(x,y) = d0;
	normals_out(x,y) = normals_in(x,y);
	if (d0 < camera.minDepth || d0 > camera.maxDepth) return;
	float3 X = camera.screenToCam((int)(x),(int)(y),d0);

	//uchar4 c0 = colour_in.tex2D(x, y);
	float smooth = smoothing(x,y);

    // Neighbourhood
    for (int v=-SEARCH_RADIUS; v<=SEARCH_RADIUS; ++v) {
    for (int u=-SEARCH_RADIUS; u<=SEARCH_RADIUS; ++u) {
		const float d = depth_in.tex2D(x+u, y+v);
		if (d < camera.minDepth || d > camera.maxDepth) continue;

		// Point and normal of neighbour
		const float3 Xi = camera.screenToCam((int)(x)+u,(int)(y)+v,d);
		const float3 Ni = make_float3(normals_in.tex2D((int)(x)+u, (int)(y)+v));

		if (Ni.x+Ni.y+Ni.z == 0.0f) continue;

		// Gauss approx weighting function using point distance
		const float w = ftl::cuda::spatialWeighting(X,Xi,smooth*0.5f);

		aX += Xi*w;
		nX += Ni*w;
		contrib += w;
    }
	}

	if (contrib <= 0.0f) return;
	
	nX /= contrib;  // Weighted average normal
	aX /= contrib;  // Weighted average point (centroid)

	// Signed-Distance Field function
	float fX = nX.x * (X.x - aX.x) + nX.y * (X.y - aX.y) + nX.z * (X.z - aX.z);

	// Calculate new point using SDF function to adjust depth (and position)
	X = X - nX * fX;
	
	//uint2 screen = camera.camToScreen<uint2>(X);

    //if (screen.x < depth_out.width() && screen.y < depth_out.height()) {
    //    depth_out(screen.x,screen.y) = X.z;
	//}
	depth_out(x,y) = X.z;
	normals_out(x,y) = make_float4(nX / length(nX), 0.0f);
}

void ftl::cuda::adaptive_mls_smooth(
		ftl::cuda::TextureObject<float4> &normals_in,
		ftl::cuda::TextureObject<float4> &normals_out,
		ftl::cuda::TextureObject<float> &depth_in,
		ftl::cuda::TextureObject<float> &depth_out,
		ftl::cuda::TextureObject<float> &smoothing,
		int radius,
		const ftl::rgbd::Camera &camera,
		cudaStream_t stream) {

	const dim3 gridSize((depth_out.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth_out.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	switch (radius) {
		case 5: adaptive_mls_smooth_kernel<5><<<gridSize, blockSize, 0, stream>>>(normals_in, normals_out, depth_in, depth_out, smoothing, camera); break;
		case 4: adaptive_mls_smooth_kernel<4><<<gridSize, blockSize, 0, stream>>>(normals_in, normals_out, depth_in, depth_out, smoothing, camera); break;
		case 3: adaptive_mls_smooth_kernel<3><<<gridSize, blockSize, 0, stream>>>(normals_in, normals_out, depth_in, depth_out, smoothing, camera); break;
		case 2: adaptive_mls_smooth_kernel<2><<<gridSize, blockSize, 0, stream>>>(normals_in, normals_out, depth_in, depth_out, smoothing, camera); break;
		case 1: adaptive_mls_smooth_kernel<1><<<gridSize, blockSize, 0, stream>>>(normals_in, normals_out, depth_in, depth_out, smoothing, camera); break;
	}
	cudaSafeCall( cudaGetLastError() );


	#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	#endif
}

