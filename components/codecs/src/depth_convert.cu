#include <ftl/codecs/depth_convert_cuda.hpp>
#include "../Utils/ColorSpace.h"
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

 __device__ inline float3 depth2yuv(float depth, float maxdepth) {
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
	const dim3 gridSize((depth.cols + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth.rows + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	depth_to_vuya_kernel<<<gridSize, blockSize, 0, cv::cuda::StreamAccessor::getStream(stream)>>>(depth, rgba, maxdepth);
	cudaSafeCall( cudaGetLastError() );
}

// Planar 10bit version

__global__ void depth_to_nv12_10_kernel(cv::cuda::PtrStepSz<float> depth, ushort* luminance, ushort* chroma, int pitch, float maxdepth) {
	const unsigned int x = (blockIdx.x*blockDim.x + threadIdx.x) * 2;
	const unsigned int y = (blockIdx.y*blockDim.y + threadIdx.y) * 2;

	if (x < depth.cols && y < depth.rows) {
		float3 yuv1 = depth2yuv(depth(y,x), maxdepth);
		float3 yuv2 = depth2yuv(depth(y,x+1), maxdepth);
		float3 yuv3 = depth2yuv(depth(y+1,x), maxdepth);
		float3 yuv4 = depth2yuv(depth(y+1,x+1), maxdepth);

		// TODO: Something better than just average!
		// Bad ones are discarded anyway...
		float Ha = (yuv1.y+yuv2.y+yuv3.y+yuv4.y) / 4.0f * 255.0f;
		float Hb = (yuv1.z+yuv2.z+yuv3.z+yuv4.z) / 4.0f * 255.0f;
		
		luminance[y*pitch+x] = ushort(yuv1.x*255.0f) << 8;
		luminance[y*pitch+x+1] = ushort(yuv2.x*255.0f) << 8;
		luminance[(y+1)*pitch+x] = ushort(yuv3.x*255.0f) << 8;
		luminance[(y+1)*pitch+x+1] = ushort(yuv4.x*255.0f) << 8;

		chroma[(y/2)*pitch+x] = ushort(Ha) << 8;
		chroma[(y/2)*pitch+x+1] = ushort(Hb) << 8;
	}
}

void ftl::cuda::depth_to_nv12_10(const cv::cuda::PtrStepSz<float> &depth, ushort* luminance, ushort* chroma, int pitch, float maxdepth, cv::cuda::Stream &stream) {
	const dim3 gridSize((depth.cols/2 + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth.rows/2 + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

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

 __device__ inline float yuv2depth(float L, float Ha, float Hb) {
	const float p = P;
        
	int m = int(floor(4.0f*(L/p) - 0.5f)) % 4;
	float L0 = L - fmodf((L-(p/8.0f)), p) + (p/4.0f)*float(m) - (p/8.0f);

	float s = 0.0f;
	if (m == 0) s = (p/2.0f)*Ha;
	if (m == 1) s = (p/2.0f)*Hb;
	if (m == 2) s = (p/2.0f)*(1.0f - Ha);
	if (m == 3) s = (p/2.0f)*(1.0f - Hb);

	return (L0+s);
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
	const dim3 gridSize((depth.cols + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth.rows + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	vuya_to_depth_kernel<<<gridSize, blockSize, 0, cv::cuda::StreamAccessor::getStream(stream)>>>(depth, rgba, maxdepth);
	cudaSafeCall( cudaGetLastError() );
}

// ==== Planar version =========================================================

template <typename T>
struct T2 {
	T x;
	T y;
};

template <typename T>
__device__ inline float2 readChroma(const T* __restrict__ chroma, int pitch, uint x, uint y) {
	T2<T> c = *(T2<T>*)(&chroma[(y/2)*pitch+x]);
	return {
		float(round8(c.x)) / 255.0f,
		float(round8(c.y)) / 255.0f
	};
}

template <typename T>
__device__ inline float2 bilinChroma(const T* __restrict__ chroma, int pitch, uint x, uint y, const float2 &D, int dx, int dy, int width, int height) {
	if (uint(x+dx) >= width || uint(y+dy) >= height) return D;

	float2 A = readChroma(chroma, pitch, x+dx, y+dy);
	float2 B = readChroma(chroma, pitch, x, y+dy);
	float2 C = readChroma(chroma, pitch, x+dx, y);
	return {
		0.0625f*A.x + 0.1875f*B.x + 0.1875f*C.x + 0.5625f*D.x,
		0.0625f*A.y + 0.1875f*B.y + 0.1875f*C.y + 0.5625f*D.y
	};
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
 template <typename T>
 __global__ void vuya_to_depth_kernel(cv::cuda::PtrStepSz<float> depth, const T* __restrict__ luminance, const T* __restrict__ chroma, int pitch, float maxdepth) {
	const unsigned int x = (blockIdx.x*blockDim.x + threadIdx.x) * 2;
	const unsigned int y = (blockIdx.y*blockDim.y + threadIdx.y) * 2;

	if (x < depth.cols && y < depth.rows) {
		const float L[4] = {
			float(round8(luminance[y*pitch+x])),
			float(round8(luminance[y*pitch+x+1])),
			float(round8(luminance[(y+1)*pitch+x])),
			float(round8(luminance[(y+1)*pitch+x+1]))
		};

		bool consistent = true;
		// TODO: Check second derivative to allow for non frontal planes.
		if (fabs(L[0]-L[1]) > 2.0f) consistent = false;
		if (fabs(L[0]-L[2]) > 2.0f) consistent = false;
		if (fabs(L[3]-L[1]) > 2.0f) consistent = false;
		if (fabs(L[3]-L[2]) > 2.0f) consistent = false;

		//bool consistent = s_consistent[threadIdx.x+1 + (threadIdx.y+1)*(SIZE+2)];

		// Only the top 8 bits contain any data
		float2 H = readChroma(chroma, pitch, x, y);

		float d[4] = {0.0f, 0.0f, 0.0f, 0.0f};

		// TODO: Preload chroma? Following code is inefficient
		// Note: the full version needs to also consider if the neighbour chroma
		// block is consistent. However, since we always need to discard pixels
		// at discontinuities anyway, we can just not care about it here. This
		// significantly simplifies the situation.

		if (consistent) {
			float2 H2;
			H2 = bilinChroma(chroma, pitch, x, y, H, -2, -2, depth.cols, depth.rows);
			d[0] = yuv2depth(L[0] / 255.0f, H2.x, H2.y) * maxdepth;
			H2 = bilinChroma(chroma, pitch, x, y, H, 2, -2, depth.cols, depth.rows);
			d[1] = yuv2depth(L[1] / 255.0f, H2.x, H2.y) * maxdepth;
			H2 = bilinChroma(chroma, pitch, x, y, H, -2, 2, depth.cols, depth.rows);
			d[2] = yuv2depth(L[2] / 255.0f, H2.x, H2.y) * maxdepth;
			H2 = bilinChroma(chroma, pitch, x, y, H, 2, 2, depth.cols, depth.rows);
			d[3] = yuv2depth(L[3] / 255.0f, H2.x, H2.y) * maxdepth;
		}

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

	vuya_to_depth_kernel<ushort><<<gridSize, blockSize, 0, cv::cuda::StreamAccessor::getStream(stream)>>>(depth, luminance.data, chroma.data, luminance.step/sizeof(ushort), maxdepth);
	cudaSafeCall( cudaGetLastError() );
}

// ==== Decode filters =========================================================

 // Video is assumed to be 10bit encoded, returning ushort instead of uchar.
 template <int RADIUS>
 __global__ void discon_y_kernel(cv::cuda::PtrStepSz<ushort4> vuya) {
	const int x = blockIdx.x*blockDim.x + threadIdx.x;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x >= RADIUS && x < vuya.cols-RADIUS && y >= RADIUS && y < vuya.rows-RADIUS) {
        ushort4 in = vuya(y,x);
        ushort inY = round8(in.z);
        bool isdiscon = false;

        #pragma unroll
        for (int v=-1; v<=1; ++v) {
            #pragma unroll
            for (int u=-1; u<=1; ++u) {
                ushort inn = round8(vuya(y+v,x+u).z);
                isdiscon |= (abs(int(inY)-int(inn)) > 1);
            }
        }

		if (isdiscon) vuya(y,x).w = 1;
		/*if (isdiscon) {
			#pragma unroll
			for (int v=-RADIUS; v<=RADIUS; ++v) {
				#pragma unroll
				for (int u=-RADIUS; u<=RADIUS; ++u) {
					vuya(y+v,x+u).w = 1;
				}
			}
		}*/
	}
}

 // Video is assumed to be 10bit encoded, returning ushort instead of uchar.
 template <int RADIUS>
 __global__ void smooth_y_kernel(cv::cuda::PtrStepSz<ushort4> vuya) {
	const int x = blockIdx.x*blockDim.x + threadIdx.x;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x >= RADIUS && y >= RADIUS && x < vuya.cols-RADIUS-1 && y < vuya.rows-RADIUS-1) {
        ushort4 in = vuya(y,x);
		ushort best = in.z;
		float mcost = 1.e10f;

		// 1) In small radius, is there a discontinuity?
		
		if (in.w == 1) {
			//vuya(y,x).z = 30000;
			//return;

			#pragma unroll
			for (int v=-RADIUS; v<=RADIUS; ++v) {
				#pragma unroll
				for (int u=-RADIUS; u<=RADIUS; ++u) {
					ushort4 inn = vuya(y+v,x+u);
					if (inn.w == 0) {
						float err = fabsf(float(in.z) - float(inn.z));
						float cost = err*err; //err*err*dist;
						if (mcost > cost) {
							mcost = cost;
							best = inn.z;
						}
						//minerr = min(minerr, err);
						//if (err == minerr) best = inn.z;
						//miny = min(miny, inn.z);
						//sumY += float(in.z);
						//weights += 1.0f;
					}
				}
			}

			//printf("Min error: %d\n",minerr);
		
			vuya(y,x).z = best; //ushort(sumY / weights);
		}
        
		// 2) If yes, use minimum Y value
		// This acts only to remove discon values... instead a correction is needed
		// Weight based on distance from discon and difference from current value
		//     - points further from discon are more reliable
		//     - most similar Y is likely to be correct depth.
		//     - either use Y with strongest weight or do weighted average.
        //if (isdiscon) in.z = maxy;
        //if (isdiscon) vuya(y,x) = in;
	}
}

void ftl::cuda::smooth_y(const cv::cuda::PtrStepSz<ushort4> &rgba, cv::cuda::Stream &stream) {
	const dim3 gridSize((rgba.cols + T_PER_BLOCK - 1)/T_PER_BLOCK, (rgba.rows + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    discon_y_kernel<1><<<gridSize, blockSize, 0, cv::cuda::StreamAccessor::getStream(stream)>>>(rgba);
	smooth_y_kernel<6><<<gridSize, blockSize, 0, cv::cuda::StreamAccessor::getStream(stream)>>>(rgba);
	cudaSafeCall( cudaGetLastError() );
}

// ==== Colour conversions =====================================================

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

/*static void SetMatRgb2Yuv(int iMatrix) {
    float wr, wb;
    int black, white, max;
    GetConstants(iMatrix, wr, wb, black, white, max);
    float mat[3][3] = {
        wr, 1.0f - wb - wr, wb,
        -0.5f * wr / (1.0f - wb), -0.5f * (1 - wb - wr) / (1.0f - wb), 0.5f,
        0.5f, -0.5f * (1.0f - wb - wr) / (1.0f - wr), -0.5f * wb / (1.0f - wr),
    };
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            mat[i][j] = (float)(1.0 * (white - black) / max * mat[i][j]);
        }
    }
    cudaMemcpyToSymbol(matRgb2Yuv, mat, sizeof(mat));
}*/

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
