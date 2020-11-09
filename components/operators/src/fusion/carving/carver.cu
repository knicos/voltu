#include <ftl/operators/cuda/carver.hpp>
#include <cudatl/fixed.hpp>
#include <ftl/cuda/weighting.hpp>

__device__ inline float depthErrorCoef(const ftl::rgbd::Camera &cam, float disps=1.0f) {
	return disps / (cam.baseline*cam.fx);
}

// ==== Reverse Verify Result ==================================================

// No colour scale calculations
/*__global__ void reverse_check_kernel(
	float* __restrict__ depth_in,
	const float* __restrict__ depth_original,
	int pitch4,
	int opitch4,
	float4x4 transformR,
	ftl::rgbd::Camera vintrin,
	ftl::rgbd::Camera ointrin
) {
	const int x = blockIdx.x*blockDim.x + threadIdx.x;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < 0 || x >= vintrin.width || y < 0 || y >= vintrin.height) return;

	float d = depth_in[y*pitch4+x];

	const float err_coef = 0.001f; //depthErrorCoef(ointrin);
	
	int count = 10;  // Allow max 2cm of carving.
	while (--count >= 0) {
		float3 campos = transformR * vintrin.screenToCam(x,y,d);
		int2 spos = ointrin.camToScreen<int2>(campos);
		int ox = spos.x;
		int oy = spos.y;

		if (campos.z > 0.0f && ox >= 0 && ox < ointrin.width && oy >= 0 && oy < ointrin.height) {
			float d2 = depth_original[oy*opitch4+ox];

			// TODO: Threshold comes from depth error characteristics
			// If the value is significantly further then carve. Depth error
			// is not always easy to calculate, depends on source.
			if (!(d2 < ointrin.maxDepth && d2 - campos.z > d2*d2*err_coef)) break;

			d += 0.002f;
		} else break;
	}

	// Too much carving means just outright remove the point.
	depth_in[y*pitch4+x] = (count < 0) ? 0.0f : d;
}*/

__global__ void reverse_check_kernel(
	float* __restrict__ depth_in,
	const float* __restrict__ depth_original,
	//const uchar4* __restrict__ in_colour,
	//const uchar4* __restrict__ ref_colour,
	//int8_t* __restrict__ colour_scale,
	int pitch4,
	//int pitch,
	int opitch4,
	//int in_col_pitch4,
	//int o_col_pitch4,
	//int cwidth,
	//int cheight,
	float4x4 transformR,
	ftl::rgbd::Camera vintrin,
	ftl::rgbd::Camera ointrin
) {
	const int x = blockIdx.x*blockDim.x + threadIdx.x;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < 0 || x >= vintrin.width || y < 0 || y >= vintrin.height) return;

	float d = depth_in[y*pitch4+x];

	const float err_coef = depthErrorCoef(ointrin);

	int ox = 0;
	int oy = 0;

	bool match = false;
	
	int count = 10;  // Allow max 2cm of carving.
	while (--count >= 0) {
		float3 campos = transformR * vintrin.screenToCam(x,y,d);
		int2 spos = ointrin.camToScreen<int2>(campos);
		ox = spos.x;
		oy = spos.y;

		if (campos.z > 0.0f && ox >= 0 && ox < ointrin.width && oy >= 0 && oy < ointrin.height) {
			float d2 = depth_original[oy*opitch4+ox];

			// TODO: Threshold comes from depth error characteristics
			// If the value is significantly further then carve. Depth error
			// is not always easy to calculate, depends on source.
			// FIXME: Use length between 3D points, not depth?
			if (!(d2 < ointrin.maxDepth && d2 - campos.z > d2*d2*err_coef)) {
				match = fabsf(campos.z - d2) < d2*d2*err_coef; break;
			}

			d += 0.002f;  // TODO: Should this be += error or what?
		} else break;
	}

	// We found a match, so do a colour check
	//float idiff = 127.0f;
	//if (match) {
	/*	// Generate colour scaling
		const float ximgscale = float(cwidth) / float(ointrin.width);
		ox = float(ox) * ximgscale;
		const float yimgscale = float(cheight) / float(ointrin.height);
		oy = float(oy) * yimgscale;

		int cy = float(y) * yimgscale;
		int cx = float(x) * ximgscale;

		const uchar4 vcol = in_colour[cy*in_col_pitch4+cx];
		const uchar4 ocol = (match) ? ref_colour[oy*o_col_pitch4+ox] : vcol;

		float i1 = (0.2126f*float(vcol.z) + 0.7152f*float(vcol.y) + 0.0722f*float(vcol.x));
		float i2 = (0.2126f*float(ocol.z) + 0.7152f*float(ocol.y) + 0.0722f*float(ocol.x));
		idiff = i2-i1;

		//const float scaleX = (vcol.x == 0) ? 1.0f : float(ocol.x) / float(vcol.x);
		//const float scaleY = (vcol.y == 0) ? 1.0f : float(ocol.y) / float(vcol.y);
		//const float scaleZ = (vcol.z == 0) ? 1.0f : float(ocol.z) / float(vcol.z);
		//scale = (0.2126f*scaleZ + 0.7152f*scaleY + 0.0722f*scaleX);
	//}
	colour_scale[x+pitch*y] = int8_t(max(-127.0f,min(127.0f,idiff)));*/

	// Too much carving means just outright remove the point.
	depth_in[y*pitch4+x] = (count < 0) ? 0.0f : d;
}

void ftl::cuda::depth_carve(
	cv::cuda::GpuMat &depth_in,
	const cv::cuda::GpuMat &depth_original,
	//const cv::cuda::GpuMat &in_colour,
	//const cv::cuda::GpuMat &ref_colour,
	//cv::cuda::GpuMat &colour_scale,
	const float4x4 &transformR,
	const ftl::rgbd::Camera &vintrin,
	const ftl::rgbd::Camera &ointrin,
	cudaStream_t stream)
{
	static constexpr int THREADS_X = 16;
	static constexpr int THREADS_Y = 8;

	const dim3 gridSize((depth_in.cols + THREADS_X - 1)/THREADS_X, (depth_in.rows + THREADS_Y - 1)/THREADS_Y);
	const dim3 blockSize(THREADS_X, THREADS_Y);

	//colour_scale.create(depth_in.size(), CV_8U);

	reverse_check_kernel<<<gridSize, blockSize, 0, stream>>>(
		depth_in.ptr<float>(),
		depth_original.ptr<float>(),
		//in_colour.ptr<uchar4>(),
		//ref_colour.ptr<uchar4>(),
		//colour_scale.ptr<int8_t>(),
		depth_in.step1(),
		//colour_scale.step1(),
		depth_original.step1(),
		//in_colour.step1()/4,
		//ref_colour.step1()/4,
		//in_colour.cols,
		//in_colour.rows,
		transformR,
		vintrin, ointrin);

	cudaSafeCall( cudaGetLastError() );
}

// ==== Apply colour scale =====================================================

template <int RADIUS>
__global__ void apply_colour_scaling_kernel(
	const int8_t* __restrict__ scale,
	uchar4* __restrict__ colour,
	int spitch,
	int cpitch,
	int swidth,
	int sheight,
	int cwidth,
	int cheight
) {
	const int x = blockIdx.x*blockDim.x + threadIdx.x;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x >= 0 && x < cwidth && y >= 0 && y < cheight) {
		int sx = (float(swidth) / float(cwidth)) * float(x);
		int sy = (float(sheight) / float(cheight)) * float(y);

		float s = 0.0f;
		int count = 0;
		//float mindiff = 100.0f;

		for (int v=-RADIUS; v<=RADIUS; ++v) {
			#pragma unroll
			for (int u=-RADIUS; u<=RADIUS; ++u) {
				float ns = (sx >= RADIUS && sy >= RADIUS && sx < swidth-RADIUS && sy < sheight-RADIUS) ? scale[sx+u+(sy+v)*spitch] : 0.0f;
				if (fabsf(ns) < 30) {
					s += ns;
					++count;
				}
			}
		}

		if (count > 0) s /= float(count);

		uchar4 c = colour[x+y*cpitch];
		colour[x+y*cpitch] = make_uchar4(
			max(0.0f, min(255.0f, float(c.x) + s)),
			max(0.0f, min(255.0f, float(c.y) + s)),
			max(0.0f, min(255.0f, float(c.z) + s)),
			255.0f
		);
	}
}

void ftl::cuda::apply_colour_scaling(
	const cv::cuda::GpuMat &scale,
	cv::cuda::GpuMat &colour,
	int radius,
	cudaStream_t stream)
{
	static constexpr int THREADS_X = 16;
	static constexpr int THREADS_Y = 8;

	const dim3 gridSize((colour.cols + THREADS_X - 1)/THREADS_X, (colour.rows + THREADS_Y - 1)/THREADS_Y);
	const dim3 blockSize(THREADS_X, THREADS_Y);

	apply_colour_scaling_kernel<2><<<gridSize, blockSize, 0, stream>>>(
		scale.ptr<int8_t>(),
		colour.ptr<uchar4>(),
		scale.step1(),
		colour.step1()/4,
		scale.cols,
		scale.rows,
		colour.cols,
		colour.rows
	);

	cudaSafeCall( cudaGetLastError() );
}
