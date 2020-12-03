/**
 * @file depth_convert.cu
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#include <ftl/codecs/depth_convert_cuda.hpp>
#include "../Utils/ColorSpace.h"
#include <opencv2/core/cuda_stream_accessor.hpp>

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

  // Assumes 8 (256) bit output channels and 14bit (16384) depth
  static constexpr float P = (2.0f * 256.0f) / 16384.0f;

  /* Convert single float to L Ha Hb. */
 __device__ inline float3 depth2yuv(float depth, float maxdepth) {
	 // Normalise
	float d = max(0.0f,depth);
	if (d >= maxdepth) d = 0.0f;
	float L = d / maxdepth;
	const float p = P;
	
	float Ha1 = fmodf((L / (p/2.0f)), 2.0f);
	float Ha = (Ha1 <= 1.0f) ? Ha1 : 2.0f - Ha1;

	float Hb1 = fmodf(((L - (p/4.0f)) / (p/2.0f)), 2.0f);
	float Hb = (Hb1 <= 1.0f) ? Hb1 : 2.0f - Hb1;

	return {L, Ha, Hb};
 }

__global__ void depth_to_vuya_kernel(cv::cuda::PtrStepSz<float> depth, cv::cuda::PtrStepSz<uchar4> rgba, float maxdepth) {
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < depth.cols && y < depth.rows) {
		float3 yuv = depth2yuv(depth(y,x), maxdepth);
        rgba(y,x) = make_uchar4(yuv.z*255.0f,yuv.y*255.0f,yuv.x*255.0f, 0.0f);
	}
}

void ftl::cuda::depth_to_vuya(const cv::cuda::PtrStepSz<float> &depth, const cv::cuda::PtrStepSz<uchar4> &rgba, float maxdepth, cv::cuda::Stream &stream) {
	static constexpr int THREADS_X = 8;
	static constexpr int THREADS_Y = 8;

	const dim3 gridSize((depth.cols + THREADS_X - 1)/THREADS_X, (depth.rows + THREADS_Y - 1)/THREADS_Y);
    const dim3 blockSize(THREADS_X, THREADS_Y);

	depth_to_vuya_kernel<<<gridSize, blockSize, 0, cv::cuda::StreamAccessor::getStream(stream)>>>(depth, rgba, maxdepth);
	cudaSafeCall( cudaGetLastError() );
}

/* Planar 10bit version */
__global__ void depth_to_nv12_10_kernel(cv::cuda::PtrStepSz<float> depth, ushort* luminance, ushort* chroma, int pitch, float maxdepth) {
	const unsigned int x = (blockIdx.x*blockDim.x + threadIdx.x) * 2;
	const unsigned int y = (blockIdx.y*blockDim.y + threadIdx.y) * 2;

	if (x < depth.cols && y < depth.rows) {
		// Process all 4 pixels at same time, due to 4:2:0 format
		float3 yuv1 = depth2yuv(depth(y,x), maxdepth);
		float3 yuv2 = depth2yuv(depth(y,x+1), maxdepth);
		float3 yuv3 = depth2yuv(depth(y+1,x), maxdepth);
		float3 yuv4 = depth2yuv(depth(y+1,x+1), maxdepth);

		// TODO: Something better than just average!
		// Bad ones are discarded anyway...
		float Ha = (yuv1.y+yuv2.y+yuv3.y+yuv4.y) / 4.0f * 255.0f;
		float Hb = (yuv1.z+yuv2.z+yuv3.z+yuv4.z) / 4.0f * 255.0f;
		
		// Use upper 8 bits only for luma
		luminance[y*pitch+x] = ushort(yuv1.x*255.0f) << 8;
		luminance[y*pitch+x+1] = ushort(yuv2.x*255.0f) << 8;
		luminance[(y+1)*pitch+x] = ushort(yuv3.x*255.0f) << 8;
		luminance[(y+1)*pitch+x+1] = ushort(yuv4.x*255.0f) << 8;

		chroma[(y/2)*pitch+x] = ushort(Ha) << 8;
		chroma[(y/2)*pitch+x+1] = ushort(Hb) << 8;
	}
}

void ftl::cuda::depth_to_nv12_10(const cv::cuda::PtrStepSz<float> &depth, ushort* luminance, ushort* chroma, int pitch, float maxdepth, cv::cuda::Stream &stream) {
	static constexpr int THREADS_X = 8;  // TODO: (nick) tune
	static constexpr int THREADS_Y = 8;

	const dim3 gridSize((depth.cols/2 + THREADS_X - 1)/THREADS_X, (depth.rows/2 + THREADS_Y - 1)/THREADS_Y);
    const dim3 blockSize(THREADS_X, THREADS_Y);

	depth_to_nv12_10_kernel<<<gridSize, blockSize, 0, cv::cuda::StreamAccessor::getStream(stream)>>>(depth, luminance, chroma, pitch, maxdepth);
	cudaSafeCall( cudaGetLastError() );
}


// =============================================================================

// Decoding

/*
 * See: Pece F., Kautz J., Weyrich T. 2011. Adapting standard video codecs for
 *      depth streaming. Joint Virtual Reality Conference of EGVE 2011 -
 *      The 17th Eurographics Symposium on Virtual Environments, EuroVR 2011 -
 *      The 8th EuroVR (INTUITION) Conference, , pp. 59-66.
 *
 */

 __device__ inline ushort round8(ushort v) {
     return (v >> 8) + ((v >> 7) & 0x1);  // Note: Make no PSNR difference
     //return v >> 8;
 }

 __device__ inline uchar round8(uchar v) { return v; }

 /* Convert single L Ha Hb to float depth */
 __device__ inline float yuv2depth(float L, float Ha, float Hb) {
	const float p = P;
        
	int m = int(floor(4.0f*(L/p) - 0.5f)) % 4;
	float L0 = L - fmodf((L-(p/8.0f)), p) + (p/4.0f)*float(m) - (p/8.0f);

	float s = 0.0f;
	if (m == 0) s = (p/2.0f)*Ha;
	if (m == 1) s = (p/2.0f)*Hb;
	if (m == 2) s = (p/2.0f)*(1.0f - Ha);
	if (m == 3) s = (p/2.0f)*(1.0f - Hb);

	return (L0+s);  // Not denormalised!
 }

 // Video is assumed to be 10bit encoded, returning ushort instead of uchar.
__global__ void vuya_to_depth_kernel(cv::cuda::PtrStepSz<float> depth, cv::cuda::PtrStepSz<ushort4> rgba, float maxdepth) {
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < depth.cols && y < depth.rows) {
		ushort4 in = rgba(y,x);

		// Only the top 8 bits contain any data
        float L = float(round8(in.z)) / 255.0f;
        float Ha = float(round8(in.y)) / 255.0f;
		float Hb = float(round8(in.x)) / 255.0f;

        depth(y,x) = yuv2depth(L, Ha, Hb) * maxdepth;
	}
}

void ftl::cuda::vuya_to_depth(const cv::cuda::PtrStepSz<float> &depth, const cv::cuda::PtrStepSz<ushort4> &rgba, float maxdepth, cv::cuda::Stream &stream) {
	static constexpr int THREADS_X = 8;
	static constexpr int THREADS_Y = 8;

	const dim3 gridSize((depth.cols + THREADS_X - 1)/THREADS_X, (depth.rows + THREADS_Y - 1)/THREADS_Y);
    const dim3 blockSize(THREADS_X, THREADS_Y);

	vuya_to_depth_kernel<<<gridSize, blockSize, 0, cv::cuda::StreamAccessor::getStream(stream)>>>(depth, rgba, maxdepth);
	cudaSafeCall( cudaGetLastError() );
}

// ==== Planar 4:2:0 version ===================================================

// Typed pair to combine memory read
template <typename T>
struct T2 {
	T x;
	T y;
};

/* Read both chroma values together. */
template <typename T>
__device__ inline ushort2 readChroma(const T* __restrict__ chroma, int pitch, uint x, uint y) {
	T2<T> c = *(T2<T>*)(&chroma[(y/2)*pitch+x]);
	return {
		ushort(round8(c.x)),
		ushort(round8(c.y))
	};
}

__device__ inline float2 norm_float(const ushort2 &v) {
	return make_float2(float(v.x)/255.0f, float(v.y)/255.0f);
}

/*
 * Interpolate the chroma, but only if the luminance is the same. This smooths
 * the decoded output but without crossing discontinuities. If luma values are
 * themselves inconsistent then the data is marked invalid as it has been
 * corrupted by the compression.
 *
 * Unused, has been rewritten into kernel directly.
 */
template <typename T>
__device__ inline float2 bilinChroma(const T* __restrict__ chroma, const T* __restrict__ luminance, int pitch, uchar L, uint x, uint y, const ushort2 &D, int dx, int dy, int width, int height, bool consistent) {
	if (uint(x+dx) >= width || uint(y+dy) >= height) return {float(D.x)/255.0f, float(D.y)/255.0f};

	float w = 0.0f;
	float2 R = {0.0f,0.0f};

	if (round8(luminance[(y+dy)*pitch+x+dx]) == L) {
		R += 0.0625f * norm_float(readChroma(chroma, pitch, x+dx, y+dy));
		w += 0.0625f;
	}

	if (round8(luminance[(y+dy)*pitch+x]) == L) {
		R += 0.1875f * norm_float(readChroma(chroma, pitch, x, y+dy));
		w += 0.1875f;
	}

	if (round8(luminance[(y)*pitch+x+dx]) == L) {
		R += 0.1875f * norm_float(readChroma(chroma, pitch, x+dx, y));
		w += 0.1875f;
	}

	// TODO: (nick) Find way to correct data rather than discard it.
	if (consistent) {
		R.x += 0.5625f * (float(D.x) / 255.0f);
		R.y += 0.5625f * (float(D.y) / 255.0f);
		w += 0.5625f;
	}

	return R / w;  // TODO: Check W isn't 0?
}

/**
 * See: J. Korhonen, “IMPROVING IMAGE FIDELITY BY LUMA-ASSISTED CHROMA
 *    SUBSAMPLING Jari Korhonen Department of Photonics Engineering ,
 *    Technical University of Denmark.”.
 *
 * For the closest published version of the chroma upsampling applied here.
 * Difference is we can make assumptions about the depth data so have slightly
 * modified the algorithm to prevent unwanted interpolation at edges.
 */

 // Video is assumed to be 10bit encoded, returning ushort instead of uchar.
 // 4:2:0 10bit
 template <typename T, int THREADS_X, int THREADS_Y>
 __global__ void vuya_to_depth_kernel(cv::cuda::PtrStepSz<float> depth, const T* __restrict__ luminance, const T* __restrict__ chroma, int pitch, float maxdepth) {
	__shared__ uchar4 lum_s[THREADS_Y+2][64];
	__shared__ ushort2 chroma_s[THREADS_Y+2][64];
	__shared__ int consistent_s[THREADS_Y+2][64];

	for (int i=threadIdx.x + threadIdx.y*THREADS_X; i<((THREADS_X+2))*((THREADS_Y+2)); i += THREADS_X*THREADS_Y) {
		const int y = i/((THREADS_X+2));
		const int x = i%((THREADS_X+2));
		const int gx = (x + blockIdx.x*blockDim.x - 1)*2;
		const int gy = (y + blockIdx.y*blockDim.y - 1)*2;

		bool valid = (gx >= 0 && gy >= 0 && gx < depth.cols-1 && gy < depth.rows-1);

		const ushort2 v1 = (valid) ? *(const ushort2*)(&luminance[gy*pitch+gx]) : make_ushort2(0,0);
		const ushort2 v2 = (valid) ? *(const ushort2*)(&luminance[(gy+1)*pitch+gx]) : make_ushort2(0,0);
		
		short4 L = make_short4(
			round8(v1.x),
			round8(v1.y),
			round8(v2.x),
			round8(v2.y)
		);

		lum_s[y][x] = make_uchar4(L.x,L.y,L.z,L.w);
		chroma_s[y][x] = (valid) ? readChroma(chroma, pitch, gx, gy) : make_ushort2(0,0);

		bool consistent = true;
		if (abs(L.x-L.y) > 1.0f) consistent = false;
		if (abs(L.x-L.z) > 1.0f) consistent = false;
		if (abs(L.w-L.y) > 1.0f) consistent = false;
		if (abs(L.w-L.z) > 1.0f) consistent = false;
		consistent_s[y][x] = int(consistent);
	}

	__syncthreads();

	const unsigned int x = (blockIdx.x*blockDim.x + threadIdx.x)*2;
	const unsigned int y = (blockIdx.y*blockDim.y + threadIdx.y)*2;

	uchar4 L = lum_s[threadIdx.y+1][threadIdx.x+1];
	const ushort2 H = chroma_s[threadIdx.y+1][threadIdx.x+1];

	float d[4] = {0.0f, 0.0f, 0.0f, 0.0f};

	float2 H2;
	float w;
	bool consistent = consistent_s[threadIdx.y+1][threadIdx.x+1];

	// Do a bilinear interpolation of chroma, combined with a luma consistency
	// check to not smooth over boundaries, and to remove inconsistent values
	// that can be assumed to have been corrupted by the compression.

	w = 0.0f; H2 = {0.0f,0.0f};
	if (consistent_s[threadIdx.y+1-1][threadIdx.x+1-1] && L.x == lum_s[threadIdx.y+1-1][threadIdx.x+1-1].w) { H2 += 0.0625f * norm_float(chroma_s[threadIdx.y+1-1][threadIdx.x+1-1]); w += 0.0625f; }
	if (consistent_s[threadIdx.y+1-1][threadIdx.x+1] && L.x == lum_s[threadIdx.y+1-1][threadIdx.x+1].z) { H2 += 0.1875f * norm_float(chroma_s[threadIdx.y+1-1][threadIdx.x+1]); w += 0.1875f; }
	if (consistent_s[threadIdx.y+1][threadIdx.x+1-1] && L.x == lum_s[threadIdx.y+1][threadIdx.x+1-1].y) { H2 += 0.1875f * norm_float(chroma_s[threadIdx.y+1][threadIdx.x+1-1]); w += 0.1875f; }
	if (consistent) { H2 += 0.5625f * norm_float(H); w += 0.5625f; }
	if (w > 0.0f) d[0] = yuv2depth(float(L.x) / 255.0f, H2.x/w, H2.y/w) * maxdepth;

	w = 0.0f; H2 = {0.0f,0.0f};
	if (consistent_s[threadIdx.y+1-1][threadIdx.x+1+1] && L.y == lum_s[threadIdx.y+1-1][threadIdx.x+1+1].z) { H2 += 0.0625f * norm_float(chroma_s[threadIdx.y+1-1][threadIdx.x+1+1]); w += 0.0625f; }
	if (consistent_s[threadIdx.y+1-1][threadIdx.x+1] && L.y == lum_s[threadIdx.y+1-1][threadIdx.x+1].w) { H2 += 0.1875f * norm_float(chroma_s[threadIdx.y+1-1][threadIdx.x+1]); w += 0.1875f; }
	if (consistent_s[threadIdx.y+1][threadIdx.x+1+1] && L.y == lum_s[threadIdx.y+1][threadIdx.x+1+1].x) { H2 += 0.1875f * norm_float(chroma_s[threadIdx.y+1][threadIdx.x+1+1]); w += 0.1875f; }
	if (consistent) { H2 += 0.5625f * norm_float(H); w += 0.5625f; }
	if (w > 0.0f) d[1] = yuv2depth(float(L.y) / 255.0f, H2.x/w, H2.y/w) * maxdepth;

	w = 0.0f; H2 = {0.0f,0.0f};
	if (consistent_s[threadIdx.y+1+1][threadIdx.x+1-1] && L.z == lum_s[threadIdx.y+1+1][threadIdx.x+1-1].y) { H2 += 0.0625f * norm_float(chroma_s[threadIdx.y+1+1][threadIdx.x+1-1]); w += 0.0625f; }
	if (consistent_s[threadIdx.y+1+1][threadIdx.x+1] && L.z == lum_s[threadIdx.y+1+1][threadIdx.x+1].x) { H2 += 0.1875f * norm_float(chroma_s[threadIdx.y+1+1][threadIdx.x+1]); w += 0.1875f; }
	if (consistent_s[threadIdx.y+1][threadIdx.x+1-1] && L.z == lum_s[threadIdx.y+1][threadIdx.x+1-1].w) { H2 += 0.1875f * norm_float(chroma_s[threadIdx.y+1][threadIdx.x+1-1]); w += 0.1875f; }
	if (consistent) { H2 += 0.5625f * norm_float(H); w += 0.5625f; }
	if (w > 0.0f) d[2] = yuv2depth(float(L.z) / 255.0f, H2.x/w, H2.y/w) * maxdepth;

	w = 0.0f; H2 = {0.0f,0.0f};
	if (consistent_s[threadIdx.y+1+1][threadIdx.x+1+1] && L.w == lum_s[threadIdx.y+1+1][threadIdx.x+1+1].x) { H2 += 0.0625f * norm_float(chroma_s[threadIdx.y+1+1][threadIdx.x+1+1]); w += 0.0625f; }
	if (consistent_s[threadIdx.y+1+1][threadIdx.x+1] && L.w == lum_s[threadIdx.y+1+1][threadIdx.x+1].y) { H2 += 0.1875f * norm_float(chroma_s[threadIdx.y+1+1][threadIdx.x+1]); w += 0.1875f; }
	if (consistent_s[threadIdx.y+1][threadIdx.x+1+1] && L.w == lum_s[threadIdx.y+1][threadIdx.x+1+1].z) { H2 += 0.1875f * norm_float(chroma_s[threadIdx.y+1][threadIdx.x+1+1]); w += 0.1875f; }
	if (consistent_s[threadIdx.y+1][threadIdx.x+1]) { H2 += 0.5625f * norm_float(H); w += 0.5625f; }
	if (w > 0.0f) d[3] = yuv2depth(float(L.w) / 255.0f, H2.x/w, H2.y/w) * maxdepth;

	if (x < depth.cols && y < depth.rows) {
		depth(y,x) = d[0];
		depth(y,x+1) = d[1];
		depth(y+1,x) = d[2];
        depth(y+1,x+1) = d[3];
	}
}

void ftl::cuda::vuya_to_depth(const cv::cuda::PtrStepSz<float> &depth, const cv::cuda::PtrStepSz<ushort> &luminance, const cv::cuda::PtrStepSz<ushort> &chroma, float maxdepth, cv::cuda::Stream &stream) {
	static const int THREADS_X = 16;
	static const int THREADS_Y = 8;

	const dim3 gridSize((depth.cols/2 + THREADS_X - 1)/THREADS_X, (depth.rows/2 + THREADS_Y - 1)/THREADS_Y);
    const dim3 blockSize(THREADS_X, THREADS_Y);

	vuya_to_depth_kernel<ushort,THREADS_X,THREADS_Y><<<gridSize, blockSize, 0, cv::cuda::StreamAccessor::getStream(stream)>>>(depth, luminance.data, chroma.data, int(luminance.step/sizeof(ushort)), maxdepth);
	cudaSafeCall( cudaGetLastError() );
}


void ftl::cuda::smooth_y(const cv::cuda::PtrStepSz<ushort4> &rgba, cv::cuda::Stream &stream) {
	// REMOVED!!
}

// ==== Colour conversions =====================================================

// Some of the following comes from the defunct NvPipe library. It has been
// modified by us.

__constant__ float matYuv2Rgb[3][3];
__constant__ float matRgb2Yuv[3][3];

static void inline GetConstants(int iMatrix, float &wr, float &wb, int &black, int &white, int &max) {
    // Default is BT709
    wr = 0.2126f; wb = 0.0722f;
    black = 16; white = 235;
    max = 255;
    if (iMatrix == ColorSpaceStandard_BT601) {
        wr = 0.2990f; wb = 0.1140f;
    } else if (iMatrix == ColorSpaceStandard_BT2020) {
        wr = 0.2627f; wb = 0.0593f;
        // 10-bit only
        black = 64 << 6; white = 940 << 6;
        max = (1 << 16) - 1;
    }
}

// Full-range BT.709 and BT.2020 are the default matrices used for YUV to RGB conversion for 8-bit and 10/12-bit encoded streams, respectively.
// If color primaries are encoded/embedded in the bitstream, the client should use those color primaries in the conversion matrices for more accurate color reproduction.

static void SetMatYuv2Rgb(int iMatrix) {
    float wr, wb;
    int black, white, max;
    GetConstants(iMatrix, wr, wb, black, white, max);
    float mat[3][3] = {
        1.0f, 0.0f, (1.0f - wr) / 0.5f,
        1.0f, -wb * (1.0f - wb) / 0.5f / (1 - wb - wr), -wr * (1 - wr) / 0.5f / (1 - wb - wr),
        1.0f, (1.0f - wb) / 0.5f, 0.0f,
    };
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            mat[i][j] = (float)(1.0 * max / (white - black) * mat[i][j]);
        }
    }
    cudaMemcpyToSymbol(matYuv2Rgb, mat, sizeof(mat));
}

template<class T>
__device__ static T Clamp(T x, T lower, T upper) {
    return x < lower ? lower : (x > upper ? upper : x);
}

template<class Rgb, class YuvUnit>
__device__ inline Rgb YuvToRgbForPixel(YuvUnit y, YuvUnit u, YuvUnit v) {
    const int 
        low = 1 << (sizeof(YuvUnit) * 8 - 4),
        mid = 1 << (sizeof(YuvUnit) * 8 - 1);
    float fy = (int)y - low, fu = (int)u - mid, fv = (int)v - mid;
    const float maxf = (1 << sizeof(YuvUnit) * 8) - 1.0f;
    YuvUnit 
        r = (YuvUnit)Clamp(matYuv2Rgb[0][0] * fy + matYuv2Rgb[0][1] * fu + matYuv2Rgb[0][2] * fv, 0.0f, maxf),
        g = (YuvUnit)Clamp(matYuv2Rgb[1][0] * fy + matYuv2Rgb[1][1] * fu + matYuv2Rgb[1][2] * fv, 0.0f, maxf),
        b = (YuvUnit)Clamp(matYuv2Rgb[2][0] * fy + matYuv2Rgb[2][1] * fu + matYuv2Rgb[2][2] * fv, 0.0f, maxf);
    
    Rgb rgb{};
    const int nShift = abs((int)sizeof(YuvUnit) - (int)sizeof(rgb.c.r)) * 8;
    if (sizeof(YuvUnit) >= sizeof(rgb.c.r)) {
        rgb.c.r = r >> nShift;
        rgb.c.g = g >> nShift;
        rgb.c.b = b >> nShift;
    } else {
        rgb.c.r = r << nShift;
        rgb.c.g = g << nShift;
        rgb.c.b = b << nShift;
    }
    return rgb;
}

template<class YuvUnitx2, class Rgb, class RgbIntx2>
__global__ static void YuvToRgbKernel(uint8_t *pYuv, int nYuvPitch, uint8_t *pRgb, int nRgbPitch, int nWidth, int nHeight) {
    int x = (threadIdx.x + blockIdx.x * blockDim.x) * 2;
    int y = (threadIdx.y + blockIdx.y * blockDim.y) * 2;
    if (x + 1 >= nWidth || y + 1 >= nHeight) {
        return;
    }

    uint8_t *pSrc = pYuv + x * sizeof(YuvUnitx2) / 2 + y * nYuvPitch;
    uint8_t *pDst = pRgb + x * sizeof(Rgb) + y * nRgbPitch;

    YuvUnitx2 l0 = *(YuvUnitx2 *)pSrc;
    YuvUnitx2 l1 = *(YuvUnitx2 *)(pSrc + nYuvPitch);
    YuvUnitx2 ch = *(YuvUnitx2 *)(pSrc + (nHeight - y / 2) * nYuvPitch);

    *(RgbIntx2 *)pDst = RgbIntx2 {
        YuvToRgbForPixel<Rgb>(l0.x, ch.x, ch.y).d,
        YuvToRgbForPixel<Rgb>(l0.y, ch.x, ch.y).d,
    };
    *(RgbIntx2 *)(pDst + nRgbPitch) = RgbIntx2 {
        YuvToRgbForPixel<Rgb>(l1.x, ch.x, ch.y).d, 
        YuvToRgbForPixel<Rgb>(l1.y, ch.x, ch.y).d,
    };
}

template <class COLOR32>
void Nv12ToColor32(uint8_t *dpNv12, int nNv12Pitch, uint8_t *dpBgra, int nBgraPitch, int nWidth, int nHeight, int iMatrix, cudaStream_t s) {
    SetMatYuv2Rgb(iMatrix);
    YuvToRgbKernel<uchar2, COLOR32, uint2>
        <<<dim3((nWidth + 63) / 32 / 2, (nHeight + 3) / 2 / 2), dim3(32, 2)>>>
        (dpNv12, nNv12Pitch, dpBgra, nBgraPitch, nWidth, nHeight);
}

template void Nv12ToColor32<BGRA32>(uint8_t *dpNv12, int nNv12Pitch, uint8_t *dpBgra, int nBgraPitch, int nWidth, int nHeight, int iMatrix, cudaStream_t);
template void Nv12ToColor32<RGBA32>(uint8_t *dpNv12, int nNv12Pitch, uint8_t *dpBgra, int nBgraPitch, int nWidth, int nHeight, int iMatrix, cudaStream_t);

__global__
static void nv12_to_float(const uint8_t* __restrict__ src, uint32_t srcPitch, float* dst, uint32_t dstPitch, uint32_t width, uint32_t height)
{
    const uint32_t x = blockIdx.x * blockDim.x + threadIdx.x;
    const uint32_t y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x < width && y < height)
    {
        const uint32_t i = y * srcPitch + x;
        const uint32_t j = y * dstPitch + x;
		// Copy higher byte from left half of Y channel
		ushort value = (src[i]) + (src[i+width]<<8);
        //dst[j] = src[i];

        // Copy lower byte from right half of Y channel
		//dst[j + 1] = src[i + width];
		
		dst[j] = float(value) / 1000.0f;
    }
}

void ftl::cuda::nv12_to_float(const uint8_t* src, uint32_t srcPitch, float* dst, uint32_t dstPitch, uint32_t width, uint32_t height, cudaStream_t s) {
	static const int THREADS_X = 16;
	static const int THREADS_Y = 16;
	dim3 gridSize(width / THREADS_X + 1, height / THREADS_Y + 1);
    dim3 blockSize(THREADS_X, THREADS_Y);

	::nv12_to_float << <gridSize, blockSize, 0, s >> > (src, srcPitch, dst, dstPitch, width, height);
}

__global__
void float_to_nv12_16bit(const float* __restrict__ src, uint32_t srcPitch, uint8_t* dst, uint32_t dstPitch, uint32_t width, uint32_t height)
{
    const uint32_t x = blockIdx.x * blockDim.x + threadIdx.x;
    const uint32_t y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x < width && y < height)
    {
        const uint32_t i = y * srcPitch + x;
		const uint32_t j = y * dstPitch + x;
		
		float d = src[i];
		ushort ds = ushort(d*1000.0f);

        // Copy higher byte to left half of Y channel
        dst[j] = ds & 0xFF;

        // Copy lower byte to right half of Y channel
        dst[j + width] = ds >> 8;

        // Blank UV channel
        if (y < height / 2)
        {
            uint8_t* UV = dst + dstPitch * (height + y);
            UV[2 * x + 0] = 0;
            UV[2 * x + 1] = 0;
        }
    }
}

void ftl::cuda::float_to_nv12_16bit(const float* src, uint32_t srcPitch, uchar* dst, uint32_t dstPitch, uint32_t width, uint32_t height, cudaStream_t s) {
	static const int THREADS_X = 16;
	static const int THREADS_Y = 16;
	dim3 gridSize(width / THREADS_X + 1, height / THREADS_Y + 1);
    dim3 blockSize(THREADS_X, THREADS_Y);

	::float_to_nv12_16bit << <gridSize, blockSize, 0, s >> > (src, srcPitch, dst, dstPitch, width, height);
}
