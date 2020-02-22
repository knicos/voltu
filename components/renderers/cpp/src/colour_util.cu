#include "colour_cuda.hpp"

using ftl::cuda::TextureObject;

__device__ inline uchar4 make_uchar4(const uchar3 &v, uchar v2) {
	return make_uchar4(v.x, v.y, v.z, v2);
}

template <typename T, bool INVERT>
 __global__ void lut_kernel(
		const T* __restrict__ in,
		int in_pitch, 
		uchar4* __restrict__ out,
		int out_pitch,
		int width, int height,
		const uchar3* __restrict__ lut,
		float minval, float maxval) {

	__shared__ uchar4 table[256];

	int id = threadIdx.x + blockDim.x*threadIdx.y;
	table[id] = make_uchar4(lut[id], (id == 0 || id == 255) ? 0 : 255);

	__syncthreads();

	const float t = (1.0f / (maxval - minval));

	for (STRIDE_Y(y, height)) {
	for (STRIDE_X(x, width)) {
		const float i = __saturatef((float(in[x+y*in_pitch]) - minval) * t);
		out[x+y*out_pitch] = table[uchar(((INVERT) ? 1.0f-i : i)*255.0f)];
	}
	}
}

template <typename T>
void ftl::cuda::lut(TextureObject<T> &in, TextureObject<uchar4> &out,
		const cv::cuda::PtrStepSz<uchar3> &lut, float minval, float maxval,
		bool invert,
		cudaStream_t stream) {

	static constexpr int THREADS_X = 64;  // Must total 256
	static constexpr int THREADS_Y = 4;

	const dim3 gridSize(6,64);
    const dim3 blockSize(THREADS_X, THREADS_Y);

	if (invert) {
		lut_kernel<T,true><<<gridSize, blockSize, 0, stream>>>(in.devicePtr(), in.pixelPitch(), out.devicePtr(), out.pixelPitch(), out.width(), out.height(), lut.data, minval, maxval);
	} else {
		lut_kernel<T,false><<<gridSize, blockSize, 0, stream>>>(in.devicePtr(), in.pixelPitch(), out.devicePtr(), out.pixelPitch(), out.width(), out.height(), lut.data, minval, maxval);
	}
	cudaSafeCall( cudaGetLastError() );
}

template void ftl::cuda::lut<float>(TextureObject<float> &in, TextureObject<uchar4> &out,
	const cv::cuda::PtrStepSz<uchar3> &lut, float minval, float maxval, bool invert,
	cudaStream_t stream);

template void ftl::cuda::lut<short>(TextureObject<short> &in, TextureObject<uchar4> &out,
	const cv::cuda::PtrStepSz<uchar3> &lut, float minval, float maxval, bool invert,
	cudaStream_t stream);

// ==== Blending ===============================================================

__global__ void blend_alpha_kernel(
		const uchar4* __restrict__ in,
		int in_pitch, 
		uchar4* __restrict__ out,
		int out_pitch,
		int width, int height,
		float alpha) {

	for (STRIDE_Y(y, height)) {
	for (STRIDE_X(x, width)) {
		const uchar4 c1 = in[x+y*in_pitch];
		const uchar4 c2 = out[x+y*out_pitch];

		const float a = alpha*(float(c1.w)/255.0f);
		const float b = 1.0f - a;

		out[x+y*out_pitch] = make_uchar4(
			float(c1.x)*a + float(c2.x)*b,
			float(c1.y)*a + float(c2.y)*b,
			float(c1.z)*a + float(c2.z)*b,
			255.0f
		);
	}
	}
}

void ftl::cuda::blend_alpha(
		TextureObject<uchar4> &in,
		TextureObject<uchar4> &out,
		float alpha, float beta,
		cudaStream_t stream) {

	static constexpr int THREADS_X = 32;
	static constexpr int THREADS_Y = 8;

	const dim3 gridSize(6,64);
	const dim3 blockSize(THREADS_X, THREADS_Y);

	blend_alpha_kernel<<<gridSize, blockSize, 0, stream>>>(
		in.devicePtr(), in.pixelPitch(),
		out.devicePtr(), out.pixelPitch(),
		out.width(), out.height(), alpha);
	cudaSafeCall( cudaGetLastError() );
}
