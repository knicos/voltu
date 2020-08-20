/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#if !defined CUDA_DISABLER

#include "opencv2/core/cuda/common.hpp"
#include "opencv2/core/cuda/limits.hpp"

#include "disparity_bilateral_filter.hpp"

#include <ftl/cuda_common.hpp>
#include <ftl/cuda/weighting.hpp>

using namespace cv::cuda::device;
using namespace cv::cuda;
using namespace cv;

#define WARP_SIZE 32
#define FULL_MASK 0xFFFFFFFFu

#define PIXELS_PER_LOOP 16

namespace ftl { namespace cuda { namespace device
{
    namespace disp_bilateral_filter
    {

		template <typename C>
		__device__ inline uchar distance(C a, C b);

		template <>
		__device__ inline uchar distance(uchar4 a, uchar4 b) {
			uchar x = ::abs(a.x - b.x);
            uchar y = ::abs(a.y - b.y);
			uchar z = ::abs(a.z - b.z);
			return (::max(::max(x, y), z));
			/*union {
				unsigned int v;
				uchar d[4];
			};
			v = __vabsdiffs4(*(unsigned int*)&a, *(unsigned int*)&b);
            return (::max(::max(d[0], d[1]), d[2]));*/
		}

		template <>
		__device__ inline uchar distance(uchar3 a, uchar3 b) {
			uchar x = ::abs(a.x - b.x);
            uchar y = ::abs(a.y - b.y);
            uchar z = ::abs(a.z - b.z);
            return (::max(::max(x, y), z));
		}

		template <>
		__device__ inline uchar distance(uchar a, uchar b) {
			return abs(int(a)-int(b));
		}


        /*template <int channels>
        struct DistRgbMax
        {
            static __device__ __forceinline__ uchar calc(const uchar* a, const uchar* b)
            {
				// TODO: (Nick) Is this the best way to read for performance?
                uchar x = ::abs(a[0] - b[0]);
                uchar y = ::abs(a[1] - b[1]);
                uchar z = ::abs(a[2] - b[2]);
                return (::max(::max(x, y), z));
            }
		};
		
		template <>
        struct DistRgbMax<4>
        {
            static __device__ __forceinline__ uchar calc(const uchar* a, const uchar* b)
            {
				const uchar4 aa = *(uchar4*)a;
				const uchar4 bb = *(uchar4*)b;
                uchar x = ::abs(aa.x - bb.x);
                uchar y = ::abs(aa.y - bb.y);
                uchar z = ::abs(aa.z - bb.z);
                return (::max(::max(x, y), z));
            }
        };

        template <>
        struct DistRgbMax<1>
        {
            static __device__ __forceinline__ uchar calc(const uchar* a, const uchar* b)
            {
                return ::abs(a[0] - b[0]);
            }
		};*/

		__device__ inline float calc_colour_weight(int d) {
			return exp(-float(d * d) / (2.0f * 10.0f * 10.0f));
		}

		template <typename T>
		__device__ inline T Abs(T v) { return ::abs(v); }

		template <>
		__device__ inline float Abs<float>(float v) { return fabsf(v); }

        template <typename C, int CRADIUS, typename T>
        __global__ void disp_bilateral_filter(int t, const T* __restrict__ disp, T* __restrict__ dispout, size_t disp_step,
            const C* __restrict__ img, size_t img_step, int h, int w,
            const float* __restrict__ ctable_color,
            T cedge_disc, T cmax_disc)
        {
            __shared__ float s_space[(CRADIUS+1)*(CRADIUS+1)];
            __shared__ short2 s_queue[4096];  // Depends on pixels per block
            __shared__ int s_counter;

			// Create gaussian lookup for spatial weighting
			for (int i=threadIdx.x+threadIdx.y*blockDim.x; i<(CRADIUS+1)*(CRADIUS+1); ++i) {
				const int y = i / (CRADIUS+1);
				const int x = i % (CRADIUS+1);
				s_space[i] = exp(-sqrt(float(y * y) + float(x * x)) / float(CRADIUS+1));
            }
            if (threadIdx.x == 0 && threadIdx.y == 0) s_counter = 0;
			__syncthreads();

            // Check all pixels to see if they need processing
            for (STRIDE_Y(y, h)) {
            for (STRIDE_X(x, w)) {
                bool todo_pixel = false;
                if (y >= CRADIUS && y < h - CRADIUS && x >= CRADIUS && x < w - CRADIUS) {
                    T dp[5];
                    dp[0] = *(disp + (y  ) * disp_step + x + 0);
                    dp[1] = *(disp + (y-1) * disp_step + x + 0);
                    dp[2] = *(disp + (y  ) * disp_step + x - 1);
                    dp[3] = *(disp + (y+1) * disp_step + x + 0);
					dp[4] = *(disp + (y  ) * disp_step + x + 1);
					
					*(dispout + y * disp_step + x) = dp[0];

                    todo_pixel = (Abs(dp[1] - dp[0]) >= cedge_disc || Abs(dp[2] - dp[0]) >= cedge_disc || Abs(dp[3] - dp[0]) >= cedge_disc || Abs(dp[4] - dp[0]) >= cedge_disc);
                }

                // Count valid pixels and warp and allocate space for them
                const uint bal = __ballot_sync(0xFFFFFFFF, todo_pixel);
                int index = 0;
                if (threadIdx.x%32 == 0) {
                    index = atomicAdd(&s_counter, __popc(bal));
                }
                index = __shfl_sync(0xFFFFFFFF, index, 0, 32);
				index += __popc(bal >> (threadIdx.x%32)) - 1;
				if (todo_pixel) s_queue[index] = make_short2(x,y);
            }
            }

            // Switch to processing mode
            __syncthreads();

			const int counter = s_counter;

			// Stride the queue to reduce bank conflicts
			// Each thread takes a pixel that needs processing
			for (int ix=(threadIdx.x + threadIdx.y*blockDim.x); ix<counter; ix+=(blockDim.x*blockDim.y)) {
				const short2 pt = s_queue[ix];
				const int x = pt.x;
				const int y = pt.y;

				T dp[5];
				dp[0] = *(disp + (y  ) * disp_step + x + 0);
				dp[1] = *(disp + (y-1) * disp_step + x + 0);
				dp[2] = *(disp + (y  ) * disp_step + x - 1);
				dp[3] = *(disp + (y+1) * disp_step + x + 0);
				dp[4] = *(disp + (y  ) * disp_step + x + 1);

				float cost[] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

				const C ic = *(img + y * img_step + x);

				//#pragma unroll
				// Note: Don't unroll this one!
				for(int yi = -CRADIUS; yi <= CRADIUS; ++yi)
				{
					const T* disp_y = disp + (y + yi) * disp_step;

					#pragma unroll
					for(int xi = -CRADIUS; xi <= CRADIUS; ++xi) {
						const C in = *(img + (y+yi) * img_step + (xi+x));

						uchar dist_rgb = distance(ic,in);

						// The bilateral part of the filter
						const float weight = ctable_color[dist_rgb] * s_space[::abs(yi)*(CRADIUS+1) + ::abs(xi)];

						const T disp_reg = disp_y[x+xi];

						// The "joint" part checking for depth similarity
						cost[0] += ::min(cmax_disc, Abs(disp_reg - dp[0])) * weight;
						cost[1] += ::min(cmax_disc, Abs(disp_reg - dp[1])) * weight;
						cost[2] += ::min(cmax_disc, Abs(disp_reg - dp[2])) * weight;
						cost[3] += ::min(cmax_disc, Abs(disp_reg - dp[3])) * weight;
						cost[4] += ::min(cmax_disc, Abs(disp_reg - dp[4])) * weight;
					}
				}

				float minimum = cost[0];
				int id = 0;

				if (cost[1] < minimum)
				{
					minimum = cost[1];
					id = 1;
				}
				if (cost[2] < minimum)
				{
					minimum = cost[2];
					id = 2;
				}
				if (cost[3] < minimum)
				{
					minimum = cost[3];
					id = 3;
				}
				if (cost[4] < minimum)
				{
					minimum = cost[4];
					id = 4;
				}

				*(dispout + y * disp_step + x) = dp[id];
			}
        }

        template <typename T, typename C>
        void disp_bilateral_filter(cv::cuda::PtrStepSz<T> disp, cv::cuda::PtrStepSz<T> dispout, cv::cuda::PtrStepSz<C> img, int iters, const float *table_color, size_t table_step, int radius, T edge_disc, T max_disc, cudaStream_t stream)
        {
            dim3 threads(32, 8, 1);
            dim3 grid(1, 1, 1);
			grid.x = (disp.cols + 64 - 1) / 64;  // 64*64 = 4096, max pixels in block
			grid.y = (disp.rows + 64 - 1) / 64;

            T *in_ptr = disp.data;
            T *out_ptr = dispout.data;

            // Iters must be odd.
            if (iters & 0x1 == 0) iters += 1;

            switch (radius) {
                case 1  :
                    for (int i = 0; i < iters; ++i) {
                        disp_bilateral_filter<C,1><<<grid, threads, 0, stream>>>(0, in_ptr, out_ptr, disp.step/sizeof(T), (C*)img.data, img.step/sizeof(C), disp.rows, disp.cols, table_color, edge_disc, max_disc);
                        cudaSafeCall( cudaGetLastError() );
                        std::swap(in_ptr, out_ptr);
                    } break;
                case 2  :
                    for (int i = 0; i < iters; ++i) {
                        disp_bilateral_filter<C,2><<<grid, threads, 0, stream>>>(0, in_ptr, out_ptr, disp.step/sizeof(T), (C*)img.data, img.step/sizeof(C), disp.rows, disp.cols, table_color, edge_disc, max_disc);
                        cudaSafeCall( cudaGetLastError() );
                        std::swap(in_ptr, out_ptr);
                    } break;
                case 3  :
                    for (int i = 0; i < iters; ++i) {
                        disp_bilateral_filter<C,3><<<grid, threads, 0, stream>>>(0, in_ptr, out_ptr, disp.step/sizeof(T), (C*)img.data, img.step/sizeof(C), disp.rows, disp.cols, table_color, edge_disc, max_disc);
                        cudaSafeCall( cudaGetLastError() );
                        std::swap(in_ptr, out_ptr);
                    } break;
                case 4  :
                    for (int i = 0; i < iters; ++i) {
                        disp_bilateral_filter<C,4><<<grid, threads, 0, stream>>>(0, in_ptr, out_ptr, disp.step/sizeof(T), (C*)img.data, img.step/sizeof(C), disp.rows, disp.cols, table_color, edge_disc, max_disc);
                        cudaSafeCall( cudaGetLastError() );
                        std::swap(in_ptr, out_ptr);
                    } break;
                case 5  :
                    for (int i = 0; i < iters; ++i) {
                        disp_bilateral_filter<C,5><<<grid, threads, 0, stream>>>(0, in_ptr, out_ptr, disp.step/sizeof(T), (C*)img.data, img.step/sizeof(C), disp.rows, disp.cols, table_color, edge_disc, max_disc);
                        cudaSafeCall( cudaGetLastError() );
                        std::swap(in_ptr, out_ptr);
                    } break;
                case 6  :
                    for (int i = 0; i < iters; ++i) {
                        disp_bilateral_filter<C,6><<<grid, threads, 0, stream>>>(0, in_ptr, out_ptr, disp.step/sizeof(T), (C*)img.data, img.step/sizeof(C), disp.rows, disp.cols, table_color, edge_disc, max_disc);
                        cudaSafeCall( cudaGetLastError() );
                        std::swap(in_ptr, out_ptr);
                    } break;
                case 7  :
                    for (int i = 0; i < iters; ++i) {
                        disp_bilateral_filter<C,7><<<grid, threads, 0, stream>>>(0, in_ptr, out_ptr, disp.step/sizeof(T), (C*)img.data, img.step/sizeof(C), disp.rows, disp.cols, table_color, edge_disc, max_disc);
                        cudaSafeCall( cudaGetLastError() );
                        std::swap(in_ptr, out_ptr);
                    } break;
                default:
                    CV_Error(cv::Error::BadTileSize, "Unsupported kernel radius");
                }
                

            //if (stream == 0)
            //    cudaSafeCall( cudaDeviceSynchronize() );
        }

        // These are commented out since we don't use them and it slows compile
        //template void disp_bilateral_filter<uchar,uchar>(cv::cuda::PtrStepSz<uchar> disp, cv::cuda::PtrStepSz<uchar> dispout, cv::cuda::PtrStepSz<uchar> img, int iters, const float *table_color, size_t table_step, int radius, uchar, uchar, cudaStream_t stream);
		//template void disp_bilateral_filter<short,uchar>(cv::cuda::PtrStepSz<short> disp, cv::cuda::PtrStepSz<short> dispout, cv::cuda::PtrStepSz<uchar> img, int iters, const float *table_color, size_t table_step, int radius, short, short, cudaStream_t stream);
        //template void disp_bilateral_filter<float,uchar>(cv::cuda::PtrStepSz<float> disp, cv::cuda::PtrStepSz<float> dispout, cv::cuda::PtrStepSz<uchar> img, int iters, const float *table_color, size_t table_step, int radius, float, float, cudaStream_t stream);

        //template void disp_bilateral_filter<uchar,uchar3>(cv::cuda::PtrStepSz<uchar> disp, cv::cuda::PtrStepSz<uchar> dispout, cv::cuda::PtrStepSz<uchar3> img, int iters, const float *table_color, size_t table_step, int radius, uchar, uchar, cudaStream_t stream);
		//template void disp_bilateral_filter<short,uchar3>(cv::cuda::PtrStepSz<short> disp, cv::cuda::PtrStepSz<short> dispout, cv::cuda::PtrStepSz<uchar3> img, int iters, const float *table_color, size_t table_step, int radius, short, short, cudaStream_t stream);
        //template void disp_bilateral_filter<float,uchar3>(cv::cuda::PtrStepSz<float> disp, cv::cuda::PtrStepSz<float> dispout, cv::cuda::PtrStepSz<uchar3> img, int iters, const float *table_color, size_t table_step, int radius, float, float, cudaStream_t stream);

        template void disp_bilateral_filter<uchar,uchar4>(cv::cuda::PtrStepSz<uchar> disp, cv::cuda::PtrStepSz<uchar> dispout, cv::cuda::PtrStepSz<uchar4> img, int iters, const float *table_color, size_t table_step, int radius, uchar, uchar, cudaStream_t stream);
		template void disp_bilateral_filter<short,uchar4>(cv::cuda::PtrStepSz<short> disp, cv::cuda::PtrStepSz<short> dispout, cv::cuda::PtrStepSz<uchar4> img, int iters, const float *table_color, size_t table_step, int radius, short, short, cudaStream_t stream);
        template void disp_bilateral_filter<float,uchar4>(cv::cuda::PtrStepSz<float> disp, cv::cuda::PtrStepSz<float> dispout, cv::cuda::PtrStepSz<uchar4> img, int iters, const float *table_color, size_t table_step, int radius, float, float, cudaStream_t stream);
        
    } // namespace bilateral_filter
}}} // namespace ftl { namespace cuda { namespace cudev

#endif /* CUDA_DISABLER */
