#include <ftl/cuda_common.hpp>
#include <ftl/rgbd/camera.hpp>
#include <opencv2/core/cuda_stream_accessor.hpp>

#include "disparity/qsort.h"
#include "disparity/cuda.hpp"

__device__ void quicksort(float A[], size_t n)
{
	float tmp;
	#define LESS(i, j) A[i] < A[j]
	#define SWAP(i, j) tmp = A[i], A[i] = A[j], A[j] = tmp
	QSORT(n, LESS, SWAP);
}

template<typename T>
__device__  static bool inline isValidDisparity(T d) { return (0.0 < d) && (d < 256.0); } // TODO

static const int max_history = 32; // TODO dynamic shared memory

__global__ void temporal_median_filter_kernel(
	cv::cuda::PtrStepSz<float> disp,
	cv::cuda::PtrStepSz<int16_t> optflow,
	cv::cuda::PtrStepSz<float> history,
	int n_max,
	int16_t threshold, // fixed point 10.5
	float granularity  // 4 for Turing
)
{
	float sorted[max_history]; // TODO: dynamic shared memory
	for (STRIDE_Y(y, disp.rows)) {
	for (STRIDE_X(x, disp.cols)) {

		int flowx = optflow(round(y / granularity), 2 * round(x / granularity));
		int flowy = optflow(round(y / granularity), 2 * round(x / granularity) + 1);

		if ((abs(flowx) + abs(flowy)) > threshold)
		{
			// last element in history[x][y][t]
			history(y, (x + 1) * n_max - 1) = 0.0;
			return;
		}

		int count = history(y, (x + 1) * n_max - 1);
		int n = count % (n_max - 1);

		if (isValidDisparity(disp(y, x)))
		{
			history(y, (x + 1) * n_max - 1) += 1.0;
			count++;
			history(y, x * n_max + n) = disp(y, x);
		}

		int n_end = count;
		if (n_end >= n_max)	{ n_end = n_max - 1; }

		if (n_end != 0)
		{
			for (size_t i = 0; i < n_end; i++)
			{
				sorted[i] = history(y, x * n_max + i);
			}

			quicksort(sorted, n_end);
			disp(y, x) = sorted[n_end / 2];
		}
	}}
}

namespace ftl {
namespace cuda {
	
void optflow_filter(cv::cuda::GpuMat &disp, const cv::cuda::GpuMat &optflow,
					cv::cuda::GpuMat &history, int n, float threshold,
					cv::cuda::Stream &stream)
{
	dim3 grid(1, 1, 1);
	dim3 threads(128, 1, 1);
	grid.x = cv::cuda::device::divUp(disp.cols, 128);
	grid.y = cv::cuda::device::divUp(disp.rows, 1);

	// TODO: dynamic shared memory
	temporal_median_filter_kernel<<<grid, threads, 0, cv::cuda::StreamAccessor::getStream(stream)>>>
		(	disp, optflow, history, n,
			round(threshold * (1 << 5)),	// TODO: documentation; 10.5 format
			4								// TODO: (4 pixels granularity for Turing)
		);

	cudaSafeCall(cudaGetLastError());
}

}
}