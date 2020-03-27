#include <ftl/cuda_common.hpp>
#include <ftl/rgbd/camera.hpp>
#include <opencv2/core/cuda_stream_accessor.hpp>

#include "disparity/qsort.h"
#include <ftl/operators/cuda/disparity.hpp>

__device__ void quicksort(float A[], size_t n)
{
	float tmp;
	#define LESS(i, j) A[i] < A[j]
	#define SWAP(i, j) tmp = A[i], A[i] = A[j], A[j] = tmp
	QSORT(n, LESS, SWAP);
}

template<typename T>
__device__  static bool inline isValidDisparity(T d) { return d > 0.0f; }

__device__ inline float supportArea(uchar4 s) {
	const float dx = min(s.x,s.y);
	const float dy = min(s.z,s.w);
	return sqrt(dx*dx + dy*dy);
}

template <int FRAC>
__device__ inline short makeFixed(float v) {
	return static_cast<short>(v * (1<<FRAC));
}

static const int MAX_HISTORY = 16; // TODO dynamic shared memory

template <bool FILLING, int HISTORY>
__global__ void temporal_median_filter_kernel(
	cv::cuda::PtrStepSz<float> disp,
	cv::cuda::PtrStepSz<short2> optflow,
	cv::cuda::PtrStepSz<float> history,
	cv::cuda::PtrStepSz<uchar4> support,
	int n_max,
	float threshold, 
	float granularity  // 4 for Turing
)
{
	float sorted[HISTORY]; // TODO: dynamic shared memory
	for (STRIDE_Y(y, disp.rows)) {
	for (STRIDE_X(x, disp.cols)) {

		float area = supportArea(support(y,x)) / 25.0f;
		short2 flow = optflow(round(y / granularity), round(x / granularity));
		//int flowy = optflow(round(y / granularity), 2 * round(x / granularity) + 1);

		float t = area * threshold + 0.25f;  // 0.25 is the 1/4 pixel accuracy NVIDIA claim

		if (sqrt(float(flow.x*flow.x) + float(flow.y*flow.y)) > makeFixed<5>(t))  // max(abs(flow.x),abs(flow.y))
		{
			// TODO: Perhaps rather than discard it could follow the optical flow
			// This would require the generation of a depth flow also.
			// Perhaps do optical flow on the right image and compare motions,
			// small differences should indicate a change in depth. Or perhaps
			// consider any scale change? But this works less well in more cases

			// Most likely the above would have to be a totally separate process
			// since the whole history would have to be moved and the idea of
			// median breaks a little. Perhaps this operator is for static
			// areas and another operator is for motion areas.

			// last element in history[x][y][t]
			history(y, (x + 1) * n_max - 1) = 0.0;
			return;
		}

		int count = history(y, (x + 1) * n_max - 1);
		int n = count % (n_max - 1);

		const float disparity = disp(y, x);

		//if (isValidDisparity(disparity))
		{
			history(y, (x + 1) * n_max - 1) += 1.0;
			count++;
			history(y, x * n_max + n) = disparity;
		}

		if (FILLING || isValidDisparity(disparity)) {
			int n_end = count;
			if (n_end >= n_max)	{ n_end = n_max - 1; }

			if (n_end != 0)
			{
				for (size_t i = 0; i < n_end; i++)
				{
					sorted[i] = history(y, x * n_max + i);
				}

				quicksort(sorted, n_end);
				const float sd = sorted[n_end / 2];
				if (isValidDisparity(sd)) disp(y, x) = sd;
			}
		}
	}}
}

namespace ftl {
namespace cuda {
	
void optflow_filter(cv::cuda::GpuMat &disp, const cv::cuda::GpuMat &optflow,
					cv::cuda::GpuMat &history, cv::cuda::GpuMat &support,
					int n, float threshold, bool fill,
					cv::cuda::Stream &stream)
{
	dim3 grid(1, 1, 1);
	dim3 threads(128, 1, 1);
	grid.x = cv::cuda::device::divUp(disp.cols, 128);
	grid.y = cv::cuda::device::divUp(disp.rows, 1);

	if (fill) {
		temporal_median_filter_kernel<true, MAX_HISTORY><<<grid, threads, 0, cv::cuda::StreamAccessor::getStream(stream)>>>
			(	disp, optflow, history, support, n,
				threshold,	
				4								// TODO: (4 pixels granularity for Turing)
			);
	} else {
		temporal_median_filter_kernel<false, MAX_HISTORY><<<grid, threads, 0, cv::cuda::StreamAccessor::getStream(stream)>>>
			(	disp, optflow, history, support, n,
				threshold,	
				4								// TODO: (4 pixels granularity for Turing)
			);
	}

	cudaSafeCall(cudaGetLastError());
}

}
}