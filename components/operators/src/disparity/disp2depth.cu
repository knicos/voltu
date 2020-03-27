#include <ftl/cuda_common.hpp>
#include <ftl/rgbd/camera.hpp>
#include <opencv2/core/cuda_stream_accessor.hpp>
#include <ftl/operators/cuda/disparity.hpp>
#include <ftl/operators/cuda/mask.hpp>
#include <ftl/cuda/fixed.hpp>

#ifndef PINF
#define PINF __int_as_float(0x7f800000)
#endif

template<typename T_in, typename T_out>
__global__ void d2d_kernel(cv::cuda::PtrStepSz<T_in> disp, cv::cuda::PtrStepSz<T_out> depth,
		const ftl::rgbd::Camera cam, const float scale) {

	for (STRIDE_Y(v,disp.rows)) {
	for (STRIDE_X(u,disp.cols)) {
		short d = disp(v,u);
		depth(v,u) = (d == 0) ? 0.0f : ((cam.baseline*cam.fx) / ((float(d)*scale) - cam.doffs));
	}
	}
}

namespace ftl {
namespace cuda {

	template<typename T_in, typename T_out>
	void disparity_to_depth(const cv::cuda::GpuMat &disparity, cv::cuda::GpuMat &depth,
				const ftl::rgbd::Camera &c, float scale, cudaStream_t &stream) {
		dim3 grid(1,1,1);
		dim3 threads(128, 1, 1);
		grid.x = cv::cuda::device::divUp(disparity.cols, 128);
		grid.y = cv::cuda::device::divUp(disparity.rows, 1);
		d2d_kernel<T_in, T_out><<<grid, threads, 0, stream>>>(
			disparity, depth, c, scale);
		cudaSafeCall( cudaGetLastError() );
	}

	template void disparity_to_depth<short, float>(const cv::cuda::GpuMat&, cv::cuda::GpuMat&, const ftl::rgbd::Camera&, float, cudaStream_t&);
	template void disparity_to_depth<float, float>(const cv::cuda::GpuMat&, cv::cuda::GpuMat&, const ftl::rgbd::Camera&, float, cudaStream_t&);
}
}

//==============================================================================

template<typename T_in, typename T_out>
__global__ void d2drev_kernel(cv::cuda::PtrStepSz<T_in> disp, cv::cuda::PtrStepSz<T_out> depth,
	const ftl::rgbd::Camera cam, const float scale) {

	for (STRIDE_Y(v,disp.rows)) {
	for (STRIDE_X(u,disp.cols)) {
		float d = depth(v,u);
		float disparity = (d > cam.maxDepth || d < cam.minDepth) ? 0.0f : ((cam.baseline*cam.fx) / d) + cam.doffs;
		disp(v,u) = T_out(disparity*scale);
	}}
}

namespace ftl {
namespace cuda {

	template<typename T_in, typename T_out>
	void depth_to_disparity(const cv::cuda::GpuMat &depth, cv::cuda::GpuMat &disparity,
				const ftl::rgbd::Camera &c, float scale, cudaStream_t &stream) {
		dim3 grid(1,1,1);
		dim3 threads(128, 1, 1);
		grid.x = cv::cuda::device::divUp(disparity.cols, 128);
		grid.y = cv::cuda::device::divUp(disparity.rows, 1);
		d2drev_kernel<T_in, T_out><<<grid, threads, 0, stream>>>(
			disparity, depth, c, scale);
		cudaSafeCall( cudaGetLastError() );
	}

	template void depth_to_disparity<float, float>(const cv::cuda::GpuMat&, cv::cuda::GpuMat&, const ftl::rgbd::Camera&, float, cudaStream_t&);
	template void depth_to_disparity<float, short>(const cv::cuda::GpuMat&, cv::cuda::GpuMat&, const ftl::rgbd::Camera&, float, cudaStream_t&);

}
}

// =============================================================================

__global__ void remove_occ_kernel(cv::cuda::PtrStepSz<float> depth, cv::cuda::PtrStepSz<float> depthR,
	ftl::rgbd::Camera cam)
{
	for (STRIDE_Y(v,depth.rows)) {
	for (STRIDE_X(u,depth.cols)) {
		float d = depth(v,u);
		int disparity = int((d > cam.maxDepth || d < cam.minDepth) ? 0.0f : ((cam.baseline*cam.fx) / d) + cam.doffs);

		if (disparity > 0 && u-disparity > 0) {
			float dR = depthR(v,u-disparity);
			if (fabsf(d-dR) > 0.01f*d) {
				depth(v,u) = 0.0f;
			}
		}
	}
	}
}

void ftl::cuda::remove_occlusions(cv::cuda::GpuMat &depth, const cv::cuda::GpuMat &depthR,
			const ftl::rgbd::Camera &c, cudaStream_t stream) {
	dim3 grid(1,1,1);
	dim3 threads(128, 4, 1);
	grid.x = cv::cuda::device::divUp(depth.cols, 128);
	grid.y = cv::cuda::device::divUp(depth.rows, 4);
	remove_occ_kernel<<<grid, threads, 0, stream>>>(
		depth, depthR, c);
	cudaSafeCall( cudaGetLastError() );
}

__global__ void mask_occ_kernel(cv::cuda::PtrStepSz<float> depth,
	cv::cuda::PtrStepSz<float> depthR,
	cv::cuda::PtrStepSz<uchar> mask,
	ftl::rgbd::Camera cam)
{
	for (STRIDE_Y(v,depth.rows)) {
	for (STRIDE_X(u,depth.cols)) {
		float d = depth(v,u);
		int disparity = int((d > cam.maxDepth || d < cam.minDepth) ? 0.0f : ((cam.baseline*cam.fx) / d) + cam.doffs);

		if (disparity > 0 && u-disparity > 0) {
			float dR = depthR(v,u-disparity);
			if (fabsf(d-dR) > 0.01f*d) {
				mask(v,u) = mask(v,u) | ftl::cuda::Mask::kMask_Occlusion;
			}
		}
	}
	}
}

void ftl::cuda::mask_occlusions(const cv::cuda::GpuMat &depth, const cv::cuda::GpuMat &depthR,
			cv::cuda::GpuMat &mask,
			const ftl::rgbd::Camera &c, cudaStream_t stream) {
	dim3 grid(1,1,1);
	dim3 threads(128, 4, 1);
	grid.x = cv::cuda::device::divUp(depth.cols, 128);
	grid.y = cv::cuda::device::divUp(depth.rows, 4);
	mask_occ_kernel<<<grid, threads, 0, stream>>>(
		depth, depthR, mask, c);
	cudaSafeCall( cudaGetLastError() );
}


// =============================================================================

__global__ void check_reprojection_kernel(cv::cuda::PtrStepSz<short> disp,
	ftl::cuda::TextureObject<uchar4> left,
	ftl::cuda::TextureObject<uchar4> right)
{
	for (STRIDE_Y(v,disp.rows)) {
	for (STRIDE_X(u,disp.cols)) {
		const float d = float(disp(v,u)) / 16.0f;
		const float4 l = left.tex2D(float(u)+0.5f, float(v)+0.5f);

		if (d > 0) {
			const float4 r = right.tex2D(float(u-d)+0.5f, float(v)+0.5f);
			const float diff = max(fabsf(l.x-r.x),max(fabsf(l.y-r.y), fabsf(l.z-r.z)));
			if (diff > 10.0f) disp(v,u) = 0;
		}
	}
	}
}

void ftl::cuda::check_reprojection(const cv::cuda::GpuMat &disp, const ftl::cuda::TextureObject<uchar4> &left, const ftl::cuda::TextureObject<uchar4> &right, cudaStream_t stream) {
	dim3 grid(1,1,1);
	dim3 threads(128, 4, 1);
	grid.x = cv::cuda::device::divUp(disp.cols, 128);
	grid.y = cv::cuda::device::divUp(disp.rows, 4);

	check_reprojection_kernel<<<grid, threads, 0, stream>>>(disp, left, right);

	cudaSafeCall( cudaGetLastError() );
}


// =============================================================================


__global__ void show_rpe_kernel(cv::cuda::PtrStepSz<short> disp,
	cv::cuda::PtrStepSz<uchar4> left,
	cv::cuda::PtrStepSz<uchar4> right,
	float scale)
{
	for (STRIDE_Y(v,left.rows)) {
	for (STRIDE_X(u,left.cols)) {
		short d = disp(v,u) / 16;

		if (d > 0 && u-d >= 0) {
			uchar4 l = left(v,u);
			uchar4 r = right(v,u-d);
			float d = max(abs(int(l.x)-int(r.x)),max(abs(int(l.y)-int(r.y)), abs(int(l.z)-int(r.z))));

			left(v,u) = make_uchar4(0,0,min(255.0f, (d/scale) * 255.0f),255);
		}
	}
	}
}

void ftl::cuda::show_rpe(const cv::cuda::GpuMat &disp, cv::cuda::GpuMat &left, const cv::cuda::GpuMat &right,
			float scale, cudaStream_t stream) {
	dim3 grid(1,1,1);
	dim3 threads(128, 4, 1);
	grid.x = cv::cuda::device::divUp(disp.cols, 128);
	grid.y = cv::cuda::device::divUp(disp.rows, 4);
	show_rpe_kernel<<<grid, threads, 0, stream>>>(
		disp, left, right, scale);
	cudaSafeCall( cudaGetLastError() );
}

// =============================================================================


__global__ void merge_disp_kernel(cv::cuda::PtrStepSz<short> disp,
	cv::cuda::PtrStepSz<short> estimate)
{
	for (STRIDE_Y(v,disp.rows)) {
	for (STRIDE_X(u,disp.cols)) {
		short cd = disp(v,u);
		float d = fixed2float<4>((cd >= 4096) ? 0 : cd);
		float e = fixed2float<4>(estimate(v,u));

		if (e == 0.0f) d = 0.0f;
		if (fabsf(d-e) > 4.0f) d = 0.0f;
		disp(v,u) = float2fixed<4>(d);
	}
	}
}

void ftl::cuda::merge_disparities(cv::cuda::GpuMat &disp, const cv::cuda::GpuMat &estimate, cudaStream_t stream) {
	dim3 grid(1,1,1);
	dim3 threads(128, 4, 1);
	grid.x = cv::cuda::device::divUp(disp.cols, 128);
	grid.y = cv::cuda::device::divUp(disp.rows, 4);
	merge_disp_kernel<<<grid, threads, 0, stream>>>(disp, estimate);
	cudaSafeCall( cudaGetLastError() );
}

// =============================================================================


template <int MAX_DISP>
__global__ void show_disp_density_kernel(cv::cuda::PtrStepSz<short> disp,
	cv::cuda::PtrStepSz<uchar4> left,
	float scale)
{
	for (STRIDE_Y(v,disp.rows)) {
	for (STRIDE_X(u,disp.cols)) {
		short d = disp(v,u) / 16;
		int count = 0;

		for (int i=1; i<MAX_DISP; ++i) {
			if (u+i-d < disp.cols && u+i-d >= 0) {
				short dd = disp(v,u+i-d) / 16;
				if (d > 0 && dd == i) ++count;
			}
		}

		count = max(0,count-1);
		left(v,u) = make_uchar4(0,0,min(255.0f, (float(count)/4.0f) * 255.0f),255);
	}
	}
}

void ftl::cuda::show_disp_density(const cv::cuda::GpuMat &disp, cv::cuda::GpuMat &left,
			float scale, cudaStream_t stream) {
	dim3 grid(1,1,1);
	dim3 threads(128, 4, 1);
	grid.x = cv::cuda::device::divUp(disp.cols, 128);
	grid.y = cv::cuda::device::divUp(disp.rows, 4);
	show_disp_density_kernel<256><<<grid, threads, 0, stream>>>(
		disp, left, scale);
	cudaSafeCall( cudaGetLastError() );
}
