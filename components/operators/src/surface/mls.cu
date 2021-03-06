#include <ftl/operators/cuda/smoothing_cuda.hpp>

#include <ftl/cuda/weighting.hpp>

using ftl::cuda::TextureObject;

#define T_PER_BLOCK 8
#define WARP_SIZE 32



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
		TextureObject<half4> normals_in,
		TextureObject<half4> normals_out,
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
	normals_out(x,y) = normals_in(x,y);
	if (d0 < camera.minDepth || d0 > camera.maxDepth) return;
	float3 X = camera.screenToCam((int)(x),(int)(y),d0);

	float4 c0 = colour_in.tex2D((float)x+0.5f, (float)y+0.5f);

    // Neighbourhood
    for (int v=-SEARCH_RADIUS; v<=SEARCH_RADIUS; ++v) {
    for (int u=-SEARCH_RADIUS; u<=SEARCH_RADIUS; ++u) {
		const float d = depth_in.tex2D(x+u, y+v);
		if (d < camera.minDepth || d > camera.maxDepth) continue;

		// Point and normal of neighbour
		const float3 Xi = camera.screenToCam((int)(x)+u,(int)(y)+v,d);
		const float3 Ni = make_float3(normals_in.tex2D((int)(x)+u, (int)(y)+v));

		if (Ni.x+Ni.y+Ni.z == 0.0f) continue;

		const float4 c = colour_in.tex2D(float(x+u) + 0.5f, float(y+v) + 0.5f);
		const float cw = ftl::cuda::colourWeighting(c0,c,colour_smoothing);

		// Gauss approx weighting function using point distance
		const float w = ftl::cuda::spatialWeighting(X,Xi,smoothing*cw);

		aX += Xi*w;
		nX += Ni*w;
		contrib += w;
    }
	}

	if (contrib == 0.0f) return;
	
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
	normals_out(x,y) = make_half4(nX / length(nX), 0.0f);
}

void ftl::cuda::colour_mls_smooth(
		ftl::cuda::TextureObject<half4> &normals_in,
		ftl::cuda::TextureObject<half4> &normals_out,
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


// ===== Colour MLS using cross support region =================================

__device__ inline int segmentID(int u, int v) {
	if (u < 0 && v < 0) return 0x001;
	if (u > 0 && v < 0) return 0x002;
	if (u > 0 && v > 0) return 0x004;
	if (u < 0 && v > 0) return 0x008;
	return 0;
}

__device__ inline float4 make_float4(const uchar4 &v) {
	return make_float4(float(v.x), float(v.y), float(v.z), float(v.w));
}

/*
 * Smooth depth map using Moving Least Squares. This version uses colour
 * similarity weights to adjust the spatial smoothing factor. It is naive in
 * that similar colours may exist on both sides of an edge and colours at the
 * level of single pixels can be subject to noise. Colour noise should first
 * be removed from the image.
 */
 template <bool FILLING, int RADIUS>
 __global__ void colour_mls_smooth_csr_kernel(
	 	TextureObject<uchar4> region,
		TextureObject<half4> normals_in,
		TextureObject<half4> normals_out,
        TextureObject<float> depth_in,        // Virtual depth map
		TextureObject<float> depth_out,   // Accumulated output
		//TextureObject<uchar4> colour_in,
		const uchar4* __restrict__ colour_in,
		int colour_pitch,
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
	normals_out(x,y) = normals_in(x,y);
	if (d0 < camera.minDepth || d0 > camera.maxDepth) {
		if(FILLING) d0 = 0.0f;
		else return;
	}
	float3 X = camera.screenToCam((int)(x),(int)(y),d0);

	//float4 c0 = colour_in.tex2D((float)x+0.5f, (float)y+0.5f);
	float4 c0 = make_float4(colour_in[x+y*colour_pitch]);

    // Neighbourhood
	uchar4 base = region.tex2D(x,y);
	int segment_check = 0;

	// TODO: Does using a fixed loop size with range test work better?
	// Or with warp per pixel version, this would be less of a problem...
	// TODO: Do a separate vote fill step?
	for (int v=-RADIUS; v<=RADIUS; ++v) {
		uchar4 baseY = region.tex2D(x,y+v);

		#pragma unroll
		for (int u=-RADIUS; u<=RADIUS; ++u) {
			if (x+u >= 0 && x+u < depth_in.width() && y+v >= 0 && y+v < depth_in.height()) {
				const float d = depth_in.tex2D(x+u, y+v);
			//if (d > camera.minDepth && d < camera.maxDepth) {

				float w = (d <= camera.minDepth || d >= camera.maxDepth || u < -baseY.x || u > baseY.y || v < -base.z || v > base.z) ? 0.0f : 1.0f;

				// Point and normal of neighbour
				const float3 Xi = camera.screenToCam((int)(x)+u,(int)(y)+v,d);
				const float3 Ni = make_float3(normals_in.tex2D((int)(x)+u, (int)(y)+v));

				// FIXME: Ensure bad normals are removed by setting depth invalid
				//if (Ni.x+Ni.y+Ni.z == 0.0f) continue;

				//const float4 c = colour_in.tex2D(float(x+u) + 0.5f, float(y+v) + 0.5f);
				const float4 c = make_float4(colour_in[x+u+(y+v)*colour_pitch]);
				w *= ftl::cuda::colourWeighting(c0,c,colour_smoothing);

				// Allow missing point to borrow z value
				// TODO: This is a bad choice of Z! Perhaps try histogram vote approach
				//if (FILLING && d0 == 0.0f) X = camera.screenToCam((int)(x),(int)(y),Xi.z);

				// Gauss approx weighting function using point distance
				w = ftl::cuda::spatialWeighting(X,Xi,smoothing*w);

				aX += Xi*w;
				nX += Ni*w;
				contrib += w;
				//if (FILLING && w > 0.0f && v > -base.z+1 && v < base.w-1 && u > -baseY.x+1 && u < baseY.y-1) segment_check |= segmentID(u,v);
			}
		}
	}

	if (contrib > 0.0f) {
		nX /= contrib;  // Weighted average normal
		aX /= contrib;  // Weighted average point (centroid)

		if (FILLING && d0 == 0.0f) {
			if (__popc(segment_check) < 3) return;
			X = camera.screenToCam((int)(x),(int)(y),aX.z);
		}

		// Signed-Distance Field function
		float fX = nX.x * (X.x - aX.x) + nX.y * (X.y - aX.y) + nX.z * (X.z - aX.z);

		// Calculate new point using SDF function to adjust depth (and position)
		X = X - nX * fX;
		
		//uint2 screen = camera.camToScreen<uint2>(X);

		//if (screen.x < depth_out.width() && screen.y < depth_out.height()) {
		//    depth_out(screen.x,screen.y) = X.z;
		//}
		depth_out(x,y) = X.z;
		normals_out(x,y) = make_half4(nX / length(nX), 0.0f);
	}
}

void ftl::cuda::colour_mls_smooth_csr(
		ftl::cuda::TextureObject<uchar4> &region,
		ftl::cuda::TextureObject<half4> &normals_in,
		ftl::cuda::TextureObject<half4> &normals_out,
		ftl::cuda::TextureObject<float> &depth_in,
		ftl::cuda::TextureObject<float> &depth_out,
		//ftl::cuda::TextureObject<uchar4> &colour_in,
		const cv::cuda::GpuMat &colour_in,
		float smoothing,
		float colour_smoothing,
		bool filling,
		const ftl::rgbd::Camera &camera,
		cudaStream_t stream) {

	const dim3 gridSize((depth_out.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth_out.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	if (filling) {
		colour_mls_smooth_csr_kernel<true,5><<<gridSize, blockSize, 0, stream>>>(region, normals_in, normals_out, depth_in, depth_out, (uchar4*)colour_in.data, colour_in.step/4, smoothing, colour_smoothing, camera);
	} else {
		colour_mls_smooth_csr_kernel<false,5><<<gridSize, blockSize, 0, stream>>>(region, normals_in, normals_out, depth_in, depth_out, (uchar4*)colour_in.data, colour_in.step/4, smoothing, colour_smoothing, camera);
	}
		
	cudaSafeCall( cudaGetLastError() );


	#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	#endif
}


// ===== Cross Aggregate MLS ===================================================

__device__ inline float colourAsFloat(uchar4 c) {
	union {
		uchar4 col;
		float f;
	};
	col = c;
	return f;
}

__device__ inline uchar4 floatAsColour(float pf) {
	union {
		uchar4 col;
		float f;
	};
	f = pf;
	return col;
}

/*
 * Smooth depth map using Moving Least Squares. This version uses colour
 * similarity weights to adjust the spatial smoothing factor. It also uses
 * cross support windows to prevent unwanted edge crossing. This function does
 * the weighted aggregation of centroids and normals in the horizontal
 * direction.
 */
 template <int RADIUS>
 __global__ void mls_aggr_horiz_kernel(
		const uchar4* __restrict__ region,
		size_t region_pitch, 
		const half4* __restrict__ normals_in,
		size_t normals_in_pitch,
		TextureObject<half4> normals_out,
		const float* __restrict__ depth_in,        // Virtual depth map
		size_t depth_in_pitch,
		TextureObject<float4> centroid_out,   // Accumulated output
		const uchar4* __restrict__ colour_in,
		size_t colour_in_pitch,
		float smoothing,
		float colour_smoothing,
        ftl::rgbd::Camera camera) {
        
    const int x = blockIdx.x*blockDim.x + threadIdx.x;
    const int y = blockIdx.y*blockDim.y + threadIdx.y;

    if (x >= RADIUS && y >= RADIUS && x < camera.width-RADIUS && y < camera.height-RADIUS) {

		float3 aX = make_float3(0.0f,0.0f,0.0f);
		float3 nX = make_float3(0.0f,0.0f,0.0f);
		float contrib = 0.0f;

		const float d0 = depth_in[y*depth_in_pitch+x];
		const uchar4 c0 = colour_in[y*colour_in_pitch+x];

		// Note: x and y flipped as output is rotated.
		centroid_out(y,x) = make_float4(0.0f,0.0f,0.0f,colourAsFloat(c0));
		normals_out(y,x) = normals_in[y*normals_in_pitch+x];

		if (d0 > camera.minDepth && d0 < camera.maxDepth) {
		
			float3 X = camera.screenToCam((int)(x),(int)(y),d0);

			// Cross-Support Neighbourhood
			const uchar4 base = region[y*region_pitch+x];

			const half4* __restrict__ nptr = normals_in + y*normals_in_pitch + x - RADIUS;
			const float* __restrict__ dptr = depth_in + y*depth_in_pitch + x - RADIUS;
			const uchar4* __restrict__ cptr = colour_in + y*colour_in_pitch + x - RADIUS;

			#pragma unroll
			for (int u=-RADIUS; u<=RADIUS; ++u) {
				const float d = *(dptr++);

				// If outside of cross support range, set weight to 0 to ignore
				float w = (d <= camera.minDepth || d >= camera.maxDepth || u < -base.x || u > base.y) ? 0.0f : 1.0f;

				// Point and normal of neighbour
				const float3 Xi = camera.screenToCam(x+u,y,d);
				const float3 Ni = make_float3(*(nptr++));

				// Bad or missing normals should be ignored
				if (Ni.x+Ni.y+Ni.z == 0.0f) w = 0.0f;

				// Gauss approx colour weighting.
				const uchar4 c = *(cptr++);
				w *= ftl::cuda::colourWeighting2(c0,c,colour_smoothing);

				// Gauss approx weighting function using point distance
				w = ftl::cuda::spatialWeighting(X,Xi,d0*smoothing*w);

				aX += Xi*w;
				nX += Ni*w;
				contrib += w;
			}

			if (contrib > 0.0f) {
				nX /= contrib;  // Weighted average normal
				aX /= contrib;  // Weighted average point (centroid)

				// Note: x and y flipped since output is rotated 90 degrees.
				centroid_out(y,x) = make_float4(aX, colourAsFloat(c0));
				normals_out(y,x) = make_half4(nX / length(nX), 0.0f);
			}
		}
	}
}

/**
 * This function does a vertical weighted aggregation of the centroids and
 * normals generated by the horizontal aggregation.
 */
template <int RADIUS>
 __global__ void mls_aggr_vert_kernel(
		const uchar4* __restrict__ region,
		size_t region_pitch, 
		const half4* __restrict__ normals_in,
		size_t normals_in_pitch,
		TextureObject<half4> normals_out,
		const float4* __restrict__ centroid_in,        // Virtual depth map
		size_t centroid_in_pitch,
		TextureObject<float4> centroid_out,   // Accumulated output
		float smoothing,
		float colour_smoothing,
        ftl::rgbd::Camera camera) {
        
    const int x = blockIdx.x*blockDim.x + threadIdx.x;
    const int y = blockIdx.y*blockDim.y + threadIdx.y;

    if (x >= RADIUS && y >= RADIUS && x < camera.width-RADIUS && y < camera.height-RADIUS) {
		float3 aX = make_float3(0.0f,0.0f,0.0f);
		float3 nX = make_float3(0.0f,0.0f,0.0f);
		float contrib = 0.0f;
		
		const float4 cin = centroid_in[x*centroid_in_pitch+y];
		const float3 A = make_float3(cin);
		const uchar4 c0 = floatAsColour(cin.w);

		centroid_out(x,y) = make_float4(0.0f);
		normals_out(x,y) = make_half4(0.0f);
		
		if (A.z > camera.minDepth && A.z < camera.maxDepth) {
			//float3 X = camera.screenToCam((int)(x),(int)(y),d0);
			
			//const uchar4 c0 = colour_in[y*colour_in_pitch+x];

			// Cross-Support Neighbourhood
			const uchar4 base = region[y*region_pitch+x];

			const half4* __restrict__ nptr = normals_in + x*normals_in_pitch + y - RADIUS;
			const float4* __restrict__ ceptr = (centroid_in + x*centroid_in_pitch + y - RADIUS);

			#pragma unroll
			for (int v=-RADIUS; v<=RADIUS; ++v) {
				// Note: x and y flipped, input image is rotated.
				const float4 cin = *(ceptr++);
				const float3 Ai = make_float3(cin);
				const uchar4 c = floatAsColour(cin.w);

				// If outside the cross support range, set weight to 0 to ignore
				float w = (Ai.z <= camera.minDepth || Ai.z >= camera.maxDepth || v < -base.z || v > base.w) ? 0.0f : 1.0f;

				// Note: x and y flipped, input image is rotated.
				const float3 Ni = make_float3(*(nptr++));

				// Bad normals should be ignored.
				if (Ni.x+Ni.y+Ni.z == 0.0f) w = 0.0f;

				// Gauss approx colour weighting.
				//const uchar4 c = *(cptr+v*colour_in_pitch);
				w *= ftl::cuda::colourWeighting2(c0,c,colour_smoothing);

				// Gauss approx weighting function using point distance
				w = ftl::cuda::spatialWeighting(A,Ai,A.z*smoothing*w);

				aX += Ai*w;
				nX += Ni*w;
				contrib += w;
			}

			// Normalise the summed points and normals
			if (contrib > 0.0f) {
				nX /= contrib;  // Weighted average normal
				aX /= contrib;  // Weighted average point (centroid)
				centroid_out(x,y) = make_float4(aX, 0.0f);
				normals_out(x,y) = make_half4(nX / length(nX), 0.0f);
			}
		}
	}
}

/**
 * Using the aggregated centroids and normals, calculate the signed-distance-
 * field and move the depth value accordingly using the calculated normal.
 */
__global__ void mls_adjust_depth_kernel(
		TextureObject<half4> normals_in,
		TextureObject<float4> centroid_in,        // Virtual depth map
		TextureObject<float> depth_out,
		TextureObject<float> depth_in,
		ftl::rgbd::Camera camera) {
	
	const int x = blockIdx.x*blockDim.x + threadIdx.x;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x >= 0 && y >= 0 && x < depth_out.width() && y < depth_out.height()) {
		const float3 aX = make_float3(centroid_in(x,y));
		const float3 nX = make_float3(normals_in(x,y));

		float d0 = depth_in.tex2D(x, y);
		depth_out(x,y) = d0;

		if (aX.z > camera.minDepth && aX.z < camera.maxDepth) {
			float3 X = camera.screenToCam((int)(x),(int)(y),aX.z);

			// Signed-Distance Field function
			float fX = nX.x * (X.x - aX.x) + nX.y * (X.y - aX.y) + nX.z * (X.z - aX.z);

			// Calculate new point using SDF function to adjust depth (and position)
			X = X - nX * fX;
			
			depth_out(x,y) = X.z;
		}
	}
}


void ftl::cuda::mls_aggr_horiz(
		ftl::cuda::TextureObject<uchar4> &region,
		ftl::cuda::TextureObject<half4> &normals_in,
		ftl::cuda::TextureObject<half4> &normals_out,
		ftl::cuda::TextureObject<float> &depth_in,
		ftl::cuda::TextureObject<float4> &centroid_out,
		//ftl::cuda::TextureObject<uchar4> &colour_in,
		const cv::cuda::GpuMat &colour_in,
		float smoothing,
		float colour_smoothing,
		int radius,
		const ftl::rgbd::Camera &camera,
		cudaStream_t stream) {

	static constexpr int THREADS_X = 16;
	static constexpr int THREADS_Y = 16;

	const dim3 gridSize((normals_in.width() + THREADS_X - 1)/THREADS_X, (normals_in.height() + THREADS_Y - 1)/THREADS_Y);
	const dim3 blockSize(THREADS_X, THREADS_Y);

	switch(radius) {
	case 1: mls_aggr_horiz_kernel<1><<<gridSize, blockSize, 0, stream>>>(region.devicePtr(), region.pixelPitch(), normals_in.devicePtr(), normals_in.pixelPitch(), normals_out, depth_in.devicePtr(), depth_in.pixelPitch(), centroid_out, (uchar4*)colour_in.data, colour_in.step/4, smoothing, colour_smoothing, camera); break;
	case 2: mls_aggr_horiz_kernel<2><<<gridSize, blockSize, 0, stream>>>(region.devicePtr(), region.pixelPitch(), normals_in.devicePtr(), normals_in.pixelPitch(), normals_out, depth_in.devicePtr(), depth_in.pixelPitch(), centroid_out, (uchar4*)colour_in.data, colour_in.step/4, smoothing, colour_smoothing, camera); break;
	case 3: mls_aggr_horiz_kernel<3><<<gridSize, blockSize, 0, stream>>>(region.devicePtr(), region.pixelPitch(), normals_in.devicePtr(), normals_in.pixelPitch(), normals_out, depth_in.devicePtr(), depth_in.pixelPitch(), centroid_out, (uchar4*)colour_in.data, colour_in.step/4, smoothing, colour_smoothing, camera); break;
	case 5: mls_aggr_horiz_kernel<5><<<gridSize, blockSize, 0, stream>>>(region.devicePtr(), region.pixelPitch(), normals_in.devicePtr(), normals_in.pixelPitch(), normals_out, depth_in.devicePtr(), depth_in.pixelPitch(), centroid_out, (uchar4*)colour_in.data, colour_in.step/4, smoothing, colour_smoothing, camera); break;
	case 10: mls_aggr_horiz_kernel<10><<<gridSize, blockSize, 0, stream>>>(region.devicePtr(), region.pixelPitch(), normals_in.devicePtr(), normals_in.pixelPitch(), normals_out, depth_in.devicePtr(), depth_in.pixelPitch(), centroid_out, (uchar4*)colour_in.data, colour_in.step/4, smoothing, colour_smoothing, camera); break;
	case 15: mls_aggr_horiz_kernel<15><<<gridSize, blockSize, 0, stream>>>(region.devicePtr(), region.pixelPitch(), normals_in.devicePtr(), normals_in.pixelPitch(), normals_out, depth_in.devicePtr(), depth_in.pixelPitch(), centroid_out, (uchar4*)colour_in.data, colour_in.step/4, smoothing, colour_smoothing, camera); break;
	case 20: mls_aggr_horiz_kernel<20><<<gridSize, blockSize, 0, stream>>>(region.devicePtr(), region.pixelPitch(), normals_in.devicePtr(), normals_in.pixelPitch(), normals_out, depth_in.devicePtr(), depth_in.pixelPitch(), centroid_out, (uchar4*)colour_in.data, colour_in.step/4, smoothing, colour_smoothing, camera); break;
	default: return;
	}
	cudaSafeCall( cudaGetLastError() );


	#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	#endif
}

void ftl::cuda::mls_aggr_vert(
		ftl::cuda::TextureObject<uchar4> &region,
		ftl::cuda::TextureObject<half4> &normals_in,
		ftl::cuda::TextureObject<half4> &normals_out,
		ftl::cuda::TextureObject<float4> &centroid_in,
		ftl::cuda::TextureObject<float4> &centroid_out,
		float smoothing,
		float colour_smoothing,
		int radius,
		const ftl::rgbd::Camera &camera,
		cudaStream_t stream) {

	static constexpr int THREADS_X = 4;
	static constexpr int THREADS_Y = 32;

	const dim3 gridSize((normals_out.width() + THREADS_X - 1)/THREADS_X, (normals_out.height() + THREADS_Y - 1)/THREADS_Y);
	const dim3 blockSize(THREADS_X, THREADS_Y);

	switch(radius) {
	case 1: mls_aggr_vert_kernel<1><<<gridSize, blockSize, 0, stream>>>(region.devicePtr(), region.pixelPitch(), normals_in.devicePtr(), normals_in.pixelPitch(), normals_out, centroid_in.devicePtr(), centroid_in.pixelPitch(), centroid_out, smoothing, colour_smoothing, camera); break;
	case 2: mls_aggr_vert_kernel<2><<<gridSize, blockSize, 0, stream>>>(region.devicePtr(), region.pixelPitch(), normals_in.devicePtr(), normals_in.pixelPitch(), normals_out, centroid_in.devicePtr(), centroid_in.pixelPitch(), centroid_out, smoothing, colour_smoothing, camera); break;
	case 3: mls_aggr_vert_kernel<3><<<gridSize, blockSize, 0, stream>>>(region.devicePtr(), region.pixelPitch(), normals_in.devicePtr(), normals_in.pixelPitch(), normals_out, centroid_in.devicePtr(), centroid_in.pixelPitch(), centroid_out, smoothing, colour_smoothing, camera); break;
	case 5: mls_aggr_vert_kernel<5><<<gridSize, blockSize, 0, stream>>>(region.devicePtr(), region.pixelPitch(), normals_in.devicePtr(), normals_in.pixelPitch(), normals_out, centroid_in.devicePtr(), centroid_in.pixelPitch(), centroid_out, smoothing, colour_smoothing, camera); break;
	case 10: mls_aggr_vert_kernel<10><<<gridSize, blockSize, 0, stream>>>(region.devicePtr(), region.pixelPitch(), normals_in.devicePtr(), normals_in.pixelPitch(), normals_out, centroid_in.devicePtr(), centroid_in.pixelPitch(), centroid_out, smoothing, colour_smoothing, camera); break;
	case 15: mls_aggr_vert_kernel<15><<<gridSize, blockSize, 0, stream>>>(region.devicePtr(), region.pixelPitch(), normals_in.devicePtr(), normals_in.pixelPitch(), normals_out, centroid_in.devicePtr(), centroid_in.pixelPitch(), centroid_out, smoothing, colour_smoothing, camera); break;
	case 20: mls_aggr_vert_kernel<20><<<gridSize, blockSize, 0, stream>>>(region.devicePtr(), region.pixelPitch(), normals_in.devicePtr(), normals_in.pixelPitch(), normals_out, centroid_in.devicePtr(), centroid_in.pixelPitch(), centroid_out, smoothing, colour_smoothing, camera); break;
	default: return;
	}
	cudaSafeCall( cudaGetLastError() );


	#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	#endif
}

void ftl::cuda::mls_adjust_depth(
		ftl::cuda::TextureObject<half4> &normals_in,
		ftl::cuda::TextureObject<float4> &centroid_in,
		ftl::cuda::TextureObject<float> &depth_out,
		ftl::cuda::TextureObject<float> &depth_in,
		const ftl::rgbd::Camera &camera,
		cudaStream_t stream) {

	static constexpr int THREADS_X = 32;
	static constexpr int THREADS_Y = 4;

	const dim3 gridSize((depth_out.width() + THREADS_X - 1)/THREADS_X, (depth_out.height() + THREADS_Y - 1)/THREADS_Y);
	const dim3 blockSize(THREADS_X, THREADS_Y);

	mls_adjust_depth_kernel<<<gridSize, blockSize, 0, stream>>>(normals_in, centroid_in, depth_out, depth_in, camera);
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
		TextureObject<half4> normals_in,
		TextureObject<half4> normals_out,
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
	normals_out(x,y) = make_half4(nX / length(nX), 0.0f);
}

void ftl::cuda::adaptive_mls_smooth(
		ftl::cuda::TextureObject<half4> &normals_in,
		ftl::cuda::TextureObject<half4> &normals_out,
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

