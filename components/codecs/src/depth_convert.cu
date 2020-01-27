#include <ftl/codecs/depth_convert_cuda.hpp>

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
		//float d = max(0.0f,min(maxdepth,depth(y,x)));
		float d = max(0.0f,depth(y,x));
		if (d >= maxdepth) d = 0.0f;
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

 __device__ inline ushort round8(ushort v) {
     return (v >> 8) + ((v >> 7) & 0x1);  // Note: Make no PSNR difference
     //return v >> 8;
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
        bool isdiscon = false;
		//int minerr = 65000;
		ushort best = in.z;
		//ushort miny = 65000;

		//float sumY = 0.0f;
		//float weights = 0.0f;
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
						float dist = v*v + u*u;
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

void ftl::cuda::smooth_y(const cv::cuda::PtrStepSz<ushort4> &rgba, cv::cuda::Stream stream) {
	const dim3 gridSize((rgba.cols + T_PER_BLOCK - 1)/T_PER_BLOCK, (rgba.rows + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    discon_y_kernel<1><<<gridSize, blockSize, 0, cv::cuda::StreamAccessor::getStream(stream)>>>(rgba);
	smooth_y_kernel<6><<<gridSize, blockSize, 0, cv::cuda::StreamAccessor::getStream(stream)>>>(rgba);
	cudaSafeCall( cudaGetLastError() );
}
