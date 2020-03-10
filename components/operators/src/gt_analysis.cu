#include <ftl/operators/gt_cuda.hpp>

#ifndef WARP_SIZE
#define WARP_SIZE 32
#endif

#define FULL_MASK 0xffffffff

template <bool COLOUR>
__global__ void gt_anal_kernel(
	uchar4* __restrict__ colour,
	int cpitch,
	int width,
	int height,
	const float* __restrict__ depth,
	int dpitch,
	const float* __restrict__ gt,
	int gpitch,
	ftl::cuda::GTAnalysisData *out,
	ftl::rgbd::Camera cam,
	float threshold,
	float outmax
) {

	__shared__ int sinvalid;
	__shared__ int sbad;
	__shared__ int smasked;
	__shared__ float serr;

	if (threadIdx.x == 0 && threadIdx.y == 0) {
		sinvalid = 0;
		sbad = 0;
		smasked = 0;
		serr = 0.0f;
	}
	__syncthreads();

	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;

	int invalid = 0;
	int bad = 0;
	int masked = 0;
	float err = 0.0f;

	const float numer = cam.baseline*cam.fx;

	if (x < width) {
		const float* __restrict__ gt_ptr = gt+x;
		const float* __restrict__ d_ptr = depth+x;

		for (STRIDE_Y(y, height)) {
			// TODO: Verify gt and depth pitch are same
			float gtval = gt_ptr[y*dpitch];
			float dval = d_ptr[y*dpitch];

			const int tmasked = (gtval > cam.minDepth && gtval < cam.maxDepth) ? 0 : 1;
			const int tinvalid = (tmasked == 0 && (dval <= cam.minDepth || dval >= cam.maxDepth)) ? 1 : 0;

			uchar4 c = make_uchar4((tinvalid==1)?255:0,0,0,255);

			// Convert both to disparity...
			if (tinvalid == 0 && tmasked == 0) {
				dval = (numer / dval);
				gtval = (numer / gtval);

				const float e = fabsf(dval-gtval);
				bad += (e >= threshold) ? 1 : 0;
				err += e;

				if (COLOUR) {
					float nerr = min(1.0f, e / outmax);
					c.z = min(255.0f, 255.0f * nerr);
				}
			}

			invalid += tinvalid;
			masked += tmasked;

			if (COLOUR) colour[x+y*cpitch] = c;
		}
	}

	// Warp aggregate
	#pragma unroll
	for (int i = WARP_SIZE/2; i > 0; i /= 2) {
		bad += __shfl_xor_sync(FULL_MASK, bad, i, WARP_SIZE);
		invalid += __shfl_xor_sync(FULL_MASK, invalid, i, WARP_SIZE);
		masked += __shfl_xor_sync(FULL_MASK, masked, i, WARP_SIZE);
		err += __shfl_xor_sync(FULL_MASK, err, i, WARP_SIZE);
	}

	// Block aggregate
	if (threadIdx.x % WARP_SIZE == 0) {
		atomicAdd(&serr, err);
		atomicAdd(&sbad, bad);
		atomicAdd(&sinvalid, invalid);
		atomicAdd(&smasked, masked);
	}

	__syncthreads();

	// Global aggregate
	if (threadIdx.x == 0 && threadIdx.y == 0) {
		atomicAdd(&out->totalerror, serr);
		atomicAdd(&out->bad, sbad);
		atomicAdd(&out->invalid, sinvalid);
		atomicAdd(&out->masked, smasked);
	}
}

void ftl::cuda::gt_analysis(
	ftl::cuda::TextureObject<uchar4> &colour,
	ftl::cuda::TextureObject<float> &depth,
	ftl::cuda::TextureObject<float> &gt,
	ftl::cuda::GTAnalysisData *out,
	const ftl::rgbd::Camera &cam,
	float threshold,
	float outmax,
	cudaStream_t stream
) {
	static constexpr int THREADS_X = 128;
	static constexpr int THREADS_Y = 2;

	const dim3 gridSize((depth.width() + THREADS_X - 1)/THREADS_X,16);
	const dim3 blockSize(THREADS_X, THREADS_Y);

	cudaMemsetAsync(out, 0, sizeof(ftl::cuda::GTAnalysisData), stream);

	gt_anal_kernel<true><<<gridSize, blockSize, 0, stream>>>(
		colour.devicePtr(),
		colour.pixelPitch(),
		colour.width(),
		colour.height(),
		depth.devicePtr(),
		depth.pixelPitch(),
		gt.devicePtr(),
		gt.pixelPitch(),
		out,
		cam,
		threshold,
		outmax
	);
	cudaSafeCall( cudaGetLastError() );

	#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	#endif
}

void ftl::cuda::gt_analysis(
	ftl::cuda::TextureObject<float> &depth,
	ftl::cuda::TextureObject<float> &gt,
	ftl::cuda::GTAnalysisData *out,
	const ftl::rgbd::Camera &cam,
	float threshold,
	cudaStream_t stream
) {
	static constexpr int THREADS_X = 128;
	static constexpr int THREADS_Y = 2;

	const dim3 gridSize((depth.width() + THREADS_X - 1)/THREADS_X, 16);
	const dim3 blockSize(THREADS_X, THREADS_Y);

	cudaMemsetAsync(out, 0, sizeof(ftl::cuda::GTAnalysisData), stream);

	gt_anal_kernel<false><<<gridSize, blockSize, 0, stream>>>(
		nullptr,
		0,
		depth.width(),
		depth.height(),
		depth.devicePtr(),
		depth.pixelPitch(),
		gt.devicePtr(),
		gt.pixelPitch(),
		out,
		cam,
		threshold,
		1.0f
	);
	cudaSafeCall( cudaGetLastError() );

	#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	#endif
}