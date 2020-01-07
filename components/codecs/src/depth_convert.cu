#include "depth_convert_cuda.hpp"

#include <opencv2/core/cuda_stream_accessor.hpp>

#define T_PER_BLOCK 8

// Encoding

__device__ inline float clamp(float v) {
	return max(0.0f, min(1.0f, v));
}

__device__ inline float clampC(float v, float t=255.0f) {
	return max(0.0f, min(t, v));
}

/*
 * See: Pece F., Kautz J., Weyrich T. 2011. Adapting standard video codecs for
 *      depth streaming. Joint Virtual Reality Conference of EGVE 2011 -
 *      The 17th Eurographics Symposium on Virtual Environments, EuroVR 2011 -
 *      The 8th EuroVR (INTUITION) Conference, , pp. 59-66.
 *
 */

 // Assumes 8 bit output channels and 14bit depth
 static constexpr float P = (2.0f * 256.0f) / 16384.0f;

__global__ void depth_to_vuya_kernel(cv::cuda::PtrStepSz<float> depth, cv::cuda::PtrStepSz<uchar4> rgba, float maxdepth) {
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < depth.cols && y < depth.rows) {
        float d = max(0.0f,min(maxdepth,depth(y,x)));
        float L = d / maxdepth;
        const float p = P;
        
        float Ha1 = fmodf((L / (p/2.0f)), 2.0f);
        float Ha = (Ha1 <= 1.0f) ? Ha1 : 2.0f - Ha1;

        float Hb1 = fmodf(((L - (p/4.0f)) / (p/2.0f)), 2.0f);
		float Hb = (Hb1 <= 1.0f) ? Hb1 : 2.0f - Hb1;

        rgba(y,x) = make_uchar4(Hb*255.0f,Ha*255.0f,L*255.0f, 0.0f);
	}
}

void ftl::cuda::depth_to_vuya(const cv::cuda::PtrStepSz<float> &depth, const cv::cuda::PtrStepSz<uchar4> &rgba, float maxdepth, cv::cuda::Stream stream) {
	const dim3 gridSize((depth.cols + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth.rows + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	depth_to_vuya_kernel<<<gridSize, blockSize, 0, cv::cuda::StreamAccessor::getStream(stream)>>>(depth, rgba, maxdepth);
	cudaSafeCall( cudaGetLastError() );
}

// Decoding

/*
 * See: Pece F., Kautz J., Weyrich T. 2011. Adapting standard video codecs for
 *      depth streaming. Joint Virtual Reality Conference of EGVE 2011 -
 *      The 17th Eurographics Symposium on Virtual Environments, EuroVR 2011 -
 *      The 8th EuroVR (INTUITION) Conference, , pp. 59-66.
 *
 */

 // Video is assumed to be 10bit encoded, returning ushort instead of uchar.
__global__ void vuya_to_depth_kernel(cv::cuda::PtrStepSz<float> depth, cv::cuda::PtrStepSz<ushort4> rgba, float maxdepth) {
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < depth.cols && y < depth.rows) {
		ushort4 in = rgba(y,x);

		// Only the top 8 bits contain any data
        float L = float(in.z >> 8) / 255.0f;
        float Ha = float(in.y >> 8) / 255.0f;
		float Hb = float(in.x >> 8) / 255.0f;

        const float p = P;
        
        int m = int(floor(4.0f*(L/p) - 0.5f)) % 4;
        float L0 = L - fmodf((L-(p/8.0f)), p) + (p/4.0f)*float(m) - (p/8.0f);

        float s = 0.0f;
        if (m == 0) s = (p/2.0f)*Ha;
        if (m == 1) s = (p/2.0f)*Hb;
        if (m == 2) s = (p/2.0f)*(1.0f - Ha);
        if (m == 3) s = (p/2.0f)*(1.0f - Hb);

        depth(y,x) = (L0+s) * maxdepth;
	}
}

void ftl::cuda::vuya_to_depth(const cv::cuda::PtrStepSz<float> &depth, const cv::cuda::PtrStepSz<ushort4> &rgba, float maxdepth, cv::cuda::Stream stream) {
	const dim3 gridSize((depth.cols + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth.rows + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	vuya_to_depth_kernel<<<gridSize, blockSize, 0, cv::cuda::StreamAccessor::getStream(stream)>>>(depth, rgba, maxdepth);
	cudaSafeCall( cudaGetLastError() );
}
