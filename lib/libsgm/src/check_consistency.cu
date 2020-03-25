/*
Copyright 2016 Fixstars Corporation

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http ://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <libsgm.h>
#include "internal.h"

namespace {
	template<typename SRC_T, typename DST_T>
	__global__ void check_consistency_kernel(DST_T* d_leftDisp, const DST_T* d_rightDisp, const uint8_t* d_mask, int width, int height, int src_pitch, int dst_pitch, bool subpixel)  {

		const int j = blockIdx.x * blockDim.x + threadIdx.x;
		const int i = blockIdx.y * blockDim.y + threadIdx.y;

		// left-right consistency check, only on leftDisp, but could be done for rightDisp too

		uint8_t mask = d_mask[i * src_pitch + j];
		int d = d_leftDisp[i * dst_pitch + j];
		if (subpixel) {
			d >>= sgm::StereoSGM::SUBPIXEL_SHIFT;
		}
		int k = j - d;
		if (mask != 0 || d <= 0 || (k >= 0 && k < width)) {
			int diff = abs(d_rightDisp[i * dst_pitch + k] - d);
			if (mask != 0 || diff > 1) {
				// masked or left-right inconsistent pixel -> invalid
				d_leftDisp[i * dst_pitch + j] = (256 << (sgm::StereoSGM::SUBPIXEL_SHIFT+1));
			}
		}
	}
}

namespace sgm {
	namespace details {

		void check_consistency(uint8_t* d_left_disp, const uint8_t* d_right_disp, const uint8_t* d_mask, int width, int height, int depth_bits, int src_pitch, int dst_pitch, bool subpixel, cudaStream_t stream) {

			const dim3 blocks(width / 16, height / 16);
			const dim3 threads(16, 16);
			if (depth_bits == 16) {
				check_consistency_kernel<uint16_t> << < blocks, threads, 0, stream >> > (d_left_disp, d_right_disp, d_mask, width, height, src_pitch, dst_pitch, subpixel);
			}
			else if (depth_bits == 8) {
				check_consistency_kernel<uint8_t> << < blocks, threads, 0, stream >> > (d_left_disp, d_right_disp, d_mask, width, height, src_pitch, dst_pitch, subpixel);
			}

			CudaKernelCheck();
		}

		void check_consistency(uint16_t* d_left_disp, const uint16_t* d_right_disp, const uint8_t* d_mask, int width, int height, int depth_bits, int src_pitch, int dst_pitch, bool subpixel, cudaStream_t stream) {

			const dim3 blocks(width / 16, height / 16);
			const dim3 threads(16, 16);
			if (depth_bits == 16) {
				check_consistency_kernel<uint16_t> << < blocks, threads, 0, stream >> > (d_left_disp, d_right_disp, d_mask, width, height, src_pitch, dst_pitch, subpixel);
			}
			else if (depth_bits == 8) {
				check_consistency_kernel<uint8_t> << < blocks, threads, 0, stream >> > (d_left_disp, d_right_disp, d_mask, width, height, src_pitch, dst_pitch, subpixel);
			}
			
			CudaKernelCheck();	
		}

	}
}
