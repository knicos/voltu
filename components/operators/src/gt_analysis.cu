#include <ftl/operators/cuda/gt.hpp>

#ifndef WARP_SIZE
#define WARP_SIZE 32
#endif

#define FULL_MASK 0xffffffff

template <bool DISPARITY, bool VISUALISE>
__global__ void gt_analysis_kernel(
	uchar4* __restrict__ colour,
	int cpitch,
	int width,
	int height,
	const float* __restrict__ depth,
	int dpitch,
	const float* __restrict__ gt,
	int gpitch,
	const uchar* __restrict__ mask,
	int mpitch,
	ftl::cuda::GTAnalysisData *out,
	ftl::rgbd::Camera cam,
	float t_min,
	float t_max,
	uchar4 colour_value
) {
	__shared__ int svalid;
	__shared__ int smissing;
	__shared__ int smissing_masked;
	__shared__ int smasked;
	__shared__ int sgood;
	__shared__ float serr;
	__shared__ float serr_sq;

	if (threadIdx.x == 0 && threadIdx.y == 0) {
		svalid = 0;
		smissing = 0;
		smissing_masked = 0;
		smasked = 0;
		sgood = 0;
		serr = 0.0f;
		serr_sq = 0.0f;
	}
	__syncthreads();

	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;

	int valid = 0;
	int missing = 0;
	int missing_masked = 0;
	int masked = 0;
	int good = 0;
	float err = 0.0f;
	float err_sq = 0.0f;

	const float numer = cam.baseline*cam.fx;

	if (x < width) {
		const float* __restrict__ gt_ptr = gt+x;
		const float* __restrict__ d_ptr = depth+x;
		const uchar* __restrict__ m_ptr = mask+x;

		for (STRIDE_Y(y, height)) {
			// TODO: Verify gt and depth pitch are same
			float gtval = gt_ptr[y*dpitch];
			float dval = d_ptr[y*dpitch];

			const int tmasked = (m_ptr[y*mpitch] == 0) ? 0 : 1;
			const int tinvalid = (dval <= cam.minDepth || dval >= cam.maxDepth) ? 1 : 0;
			const int tgtinvalid = (gtval > cam.minDepth && gtval < cam.maxDepth) ? 0 : 1;

			if (tinvalid == 0 && tgtinvalid == 0) {
				// if there is valid value in both (gt and depth)
				valid += 1;

				if (DISPARITY) {
					dval = (numer / dval);
					gtval = (numer / gtval);
				}

				const float e = fabsf(dval-gtval);

				if ((t_min < e) && (e <= t_max)) {
					good += 1;
					err += e;
					err_sq += e*e;

					if (VISUALISE) { colour[x+y*cpitch] = colour_value; }
				}
			}
			else if (tinvalid == 0 && tmasked == 1 && tgtinvalid == 1) {
				// masked and not missing (but no gt value)
				if (VISUALISE) { colour[x+y*cpitch] = {192, 0, 192, 255}; } // magenta
			}
			else if (tinvalid == 1 && (tmasked == 1 || tgtinvalid == 1)) {
				// missing and (masked or missing gt)
				if (VISUALISE) { colour[x+y*cpitch] = {0, 0, 0, 255}; } // black
				missing_masked += 1;
			}
			else if (tinvalid == 1) {
				// missing value (not masked)
				if (VISUALISE) { colour[x+y*cpitch] = {224, 32, 32, 255}; } // blue
				missing += 1;
			}

			masked += (tmasked == 1 || tgtinvalid == 1) ? 1 : 0;
		}
	}

	// Warp aggregate
	#pragma unroll
	for (int i = WARP_SIZE/2; i > 0; i /= 2) {
		valid += __shfl_xor_sync(FULL_MASK, valid, i, WARP_SIZE);
		missing += __shfl_xor_sync(FULL_MASK, missing, i, WARP_SIZE);
		missing_masked += __shfl_xor_sync(FULL_MASK, missing_masked, i, WARP_SIZE);
		masked += __shfl_xor_sync(FULL_MASK, masked, i, WARP_SIZE);
		good += __shfl_xor_sync(FULL_MASK, good, i, WARP_SIZE);
		err += __shfl_xor_sync(FULL_MASK, err, i, WARP_SIZE);
		err_sq += __shfl_xor_sync(FULL_MASK, err_sq, i, WARP_SIZE);
	}

	// Block aggregate
	if (threadIdx.x % WARP_SIZE == 0) {
		atomicAdd(&svalid, valid);
		atomicAdd(&smissing, missing);
		atomicAdd(&smissing_masked, missing_masked);
		atomicAdd(&smasked, masked);
		atomicAdd(&sgood, good);
		atomicAdd(&serr, err);
		atomicAdd(&serr_sq, err_sq);
	}

	__syncthreads();

	// Global aggregate
	if (threadIdx.x == 0 && threadIdx.y == 0) {
		atomicAdd(&out->valid, svalid);
		atomicAdd(&out->missing, smissing);
		atomicAdd(&out->missing_masked, smissing_masked);
		atomicAdd(&out->masked, smasked);
		atomicAdd(&out->good, sgood);
		atomicAdd(&out->err, serr);
		atomicAdd(&out->err_sq, serr_sq);
	}
}

void ftl::cuda::gt_analysis(
	ftl::cuda::TextureObject<uchar4> &colour,
	ftl::cuda::TextureObject<float> &depth,
	ftl::cuda::TextureObject<float> &gt,
	ftl::cuda::TextureObject<uchar> &mask,
	ftl::cuda::GTAnalysisData *out,
	const ftl::rgbd::Camera &cam,
	float t_min,
	float t_max,
	uchar4 colour_value,
	bool use_disparity,
	cudaStream_t stream
) {
	static constexpr int THREADS_X = 128;
	static constexpr int THREADS_Y = 2;

	const dim3 gridSize((depth.width() + THREADS_X - 1)/THREADS_X,16);
	const dim3 blockSize(THREADS_X, THREADS_Y);

	cudaMemsetAsync(out, 0, sizeof(ftl::cuda::GTAnalysisData), stream);

	if (use_disparity) {
		gt_analysis_kernel<true, true><<<gridSize, blockSize, 0, stream>>>(
			colour.devicePtr(),
			colour.pixelPitch(),
			colour.width(),
			colour.height(),
			depth.devicePtr(),
			depth.pixelPitch(),
			gt.devicePtr(),
			gt.pixelPitch(),
			mask.devicePtr(),
			mask.pixelPitch(),
			out,
			cam,
			t_min,
			t_max,
			colour_value
		);
	}
	else {
		gt_analysis_kernel<false, true><<<gridSize, blockSize, 0, stream>>>(
			colour.devicePtr(),
			colour.pixelPitch(),
			colour.width(),
			colour.height(),
			depth.devicePtr(),
			depth.pixelPitch(),
			gt.devicePtr(),
			gt.pixelPitch(),
			mask.devicePtr(),
			mask.pixelPitch(),
			out,
			cam,
			t_min,
			t_max,
			colour_value
		);
	}
	cudaSafeCall(cudaGetLastError());

	#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	#endif
}

void ftl::cuda::gt_analysis(
	ftl::cuda::TextureObject<float> &depth,
	ftl::cuda::TextureObject<float> &gt,
	ftl::cuda::TextureObject<uchar> &mask,
	ftl::cuda::GTAnalysisData *out,
	const ftl::rgbd::Camera &cam,
	float t_min,
	float t_max,
	bool use_disparity,
	cudaStream_t stream
) {
	static constexpr int THREADS_X = 128;
	static constexpr int THREADS_Y = 2;

	const dim3 gridSize((depth.width() + THREADS_X - 1)/THREADS_X, 16);
	const dim3 blockSize(THREADS_X, THREADS_Y);

	cudaMemsetAsync(out, 0, sizeof(ftl::cuda::GTAnalysisData), stream);

	if (use_disparity) {
		gt_analysis_kernel<true, false><<<gridSize, blockSize, 0, stream>>>(
			nullptr,
			0,
			depth.width(),
			depth.height(),
			depth.devicePtr(),
			depth.pixelPitch(),
			gt.devicePtr(),
			gt.pixelPitch(),
			mask.devicePtr(),
			mask.pixelPitch(),
			out,
			cam,
			t_min,
			t_max,
			{0,0,0,0}
		);
	}
	else {
		gt_analysis_kernel<false, false><<<gridSize, blockSize, 0, stream>>>(
			nullptr,
			0,
			depth.width(),
			depth.height(),
			depth.devicePtr(),
			depth.pixelPitch(),
			gt.devicePtr(),
			gt.pixelPitch(),
			mask.devicePtr(),
			mask.pixelPitch(),
			out,
			cam,
			t_min,
			t_max,
			{0,0,0,0}
		);
	}

	cudaSafeCall( cudaGetLastError() );

	#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	#endif
}