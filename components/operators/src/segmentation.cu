#include "segmentation_cuda.hpp"
#include <ftl/operators/mask_cuda.hpp>

#define T_PER_BLOCK 8

using ftl::cuda::TextureObject;
using ftl::cuda::Mask;

template <typename T>
__device__ inline float cross(T p1, T p2);

template <>
__device__ inline float cross<uchar4>(uchar4 p1, uchar4 p2) {
    return max(max(__sad(p1.x,p2.x,0),__sad(p1.y,p2.y,0)), __sad(p1.z,p2.z,0));
}

template <>
__device__ inline float cross<float4>(float4 p1, float4 p2) {
    return max(max(fabsf(p1.x - p2.x),fabsf(p1.y - p2.y)), fabsf(p1.z - p2.z));
}

template <>
__device__ inline float cross<float>(float p1, float p2) {
    return fabs(p1-p2);
}

template <typename T, bool SYM>
__device__ uchar4 calculate_support_region(const TextureObject<T> &img, int x, int y, float tau, int v_max, int h_max) {
    int x_min = max(0, x - h_max);
    int x_max = min(img.width()-1, static_cast<unsigned int>(x + h_max));
    int y_min = max(0, y - v_max);
    int y_max = min(img.height()-1, static_cast<unsigned int>(y + v_max));

	uchar4 result = make_uchar4(0, 0, 0, 0);

	auto colour = img.tex2D((float)x+0.5f,(float)y+0.5f);
	auto prev_colour = colour;

	int u;
    for (u=x-1; u >= x_min; --u) {
		auto next_colour = img.tex2D((float)u+0.5f,(float)y+0.5f);
        if (cross(prev_colour, next_colour) > tau) {
            result.x = x - u - 1;
            break;
		}
		prev_colour = next_colour;
	}
	if (u < x_min) result.x = x - x_min;
	
	prev_colour = colour;
    for (u=x+1; u <= x_max; ++u) {
		auto next_colour = img.tex2D((float)u+0.5f,(float)y+0.5f);
        if (cross(prev_colour, next_colour) > tau) {
            result.y = u - x - 1;
            break;
		}
		prev_colour = next_colour;
	}
	if (u > x_max) result.y = x_max - x;

	int v;
	prev_colour = colour;
    for (v=y-1; v >= y_min; --v) {
		auto next_colour = img.tex2D((float)x+0.5f,(float)v+0.5f);
        if (cross(prev_colour, next_colour) > tau) {
            result.z = y - v - 1;
            break;
		}
		prev_colour = next_colour;
	}
	if (v < y_min) result.z = y - y_min;

	prev_colour = colour;
    for (v=y+1; v <= y_max; ++v) {
		auto next_colour = img.tex2D((float)x+0.5f,(float)v+0.5f);
        if (cross(prev_colour, next_colour) > tau) {
            result.w = v - y - 1;
            break;
		}
		prev_colour = next_colour;
	}
	if (v > y_max) result.w = y_max - y;

	// Make symetric left/right and up/down
	if (SYM) {
		result.x = min(result.x, result.y);
		result.y = result.x;
		result.z = min(result.z, result.w);
		result.w = result.z;
	}
    return result;
}

__device__ uchar4 calculate_support_region(const TextureObject<uint8_t> &img, int x, int y, int v_max, int h_max) {
    int x_min = max(0, x - h_max);
    int x_max = min(img.width()-1, static_cast<unsigned int>(x + h_max));
    int y_min = max(0, y - v_max);
    int y_max = min(img.height()-1, static_cast<unsigned int>(y + v_max));

	uchar4 result = make_uchar4(0, 0, 0, 0);

	Mask m1(img.tex2D(x,y));

	int u;
    for (u=x-1; u >= x_min; --u) {
		Mask m2(img.tex2D(u,y));
        if (m2.isDiscontinuity()) {
            result.x = x - u - 1;
            break;
		}
	}
	if (u < x_min) result.x = x - x_min;
	
    for (u=x+1; u <= x_max; ++u) {
		Mask m2(img.tex2D(u,y));
        if (m2.isDiscontinuity()) {
            result.y = u - x - 1;
            break;
		}
	}
	if (u > x_max) result.y = x_max - x;

	int v;
    for (v=y-1; v >= y_min; --v) {
		Mask m2(img.tex2D(x,v));
        if (m2.isDiscontinuity()) {
            result.z = y - v - 1;
            break;
		}
	}
	if (v < y_min) result.z = y - y_min;

    for (v=y+1; v <= y_max; ++v) {
		Mask m2(img.tex2D(x,v));
        if (m2.isDiscontinuity()) {
            result.w = v - y - 1;
            break;
		}
	}
	if (v > y_max) result.w = y_max - y;

	// Make symetric left/right and up/down
	if (false) {
		result.x = min(result.x, result.y);
		result.y = result.x;
		result.z = min(result.z, result.w);
		result.w = result.z;
	}
    return result;
}

template <typename T, bool SYM>
__global__ void support_region_kernel(TextureObject<T> img, TextureObject<uchar4> region, float tau, int v_max, int h_max) {
    const int x = blockIdx.x*blockDim.x + threadIdx.x;
    const int y = blockIdx.y*blockDim.y + threadIdx.y;

    if (x < 0 || y < 0 || x >= img.width() || y >= img.height()) return;

    region(x,y) = calculate_support_region<T,SYM>(img, x, y, tau, v_max, h_max);
}

__global__ void support_region_kernel(TextureObject<uint8_t> img, TextureObject<uchar4> region, int v_max, int h_max) {
    const int x = blockIdx.x*blockDim.x + threadIdx.x;
    const int y = blockIdx.y*blockDim.y + threadIdx.y;

    if (x < 0 || y < 0 || x >= img.width() || y >= img.height()) return;

    region(x,y) = calculate_support_region(img, x, y, v_max, h_max);
}

void ftl::cuda::support_region(
        ftl::cuda::TextureObject<uchar4> &colour,
        ftl::cuda::TextureObject<uchar4> &region,
        float tau,
        int v_max,
		int h_max,
		bool sym,
        cudaStream_t stream) {

    const dim3 gridSize((region.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (region.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	if (sym) support_region_kernel<uchar4, true><<<gridSize, blockSize, 0, stream>>>(colour, region, tau, v_max, h_max);
	else support_region_kernel<uchar4, false><<<gridSize, blockSize, 0, stream>>>(colour, region, tau, v_max, h_max);
    cudaSafeCall( cudaGetLastError() );


    #ifdef _DEBUG
    cudaSafeCall(cudaDeviceSynchronize());
    #endif
}

void ftl::cuda::support_region(
		ftl::cuda::TextureObject<float> &depth,
		ftl::cuda::TextureObject<uchar4> &region,
		float tau,
		int v_max,
		int h_max,
		bool sym,
		cudaStream_t stream) {

	const dim3 gridSize((region.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (region.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	support_region_kernel<float, true><<<gridSize, blockSize, 0, stream>>>(depth, region, tau, v_max, h_max);
	cudaSafeCall( cudaGetLastError() );


	#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	#endif
}

void ftl::cuda::support_region(
		ftl::cuda::TextureObject<uint8_t> &mask,
		ftl::cuda::TextureObject<uchar4> &region,
		int v_max,
		int h_max,
		bool sym,
		cudaStream_t stream) {

	const dim3 gridSize((region.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (region.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	support_region_kernel<<<gridSize, blockSize, 0, stream>>>(mask, region, v_max, h_max);
	cudaSafeCall( cudaGetLastError() );


	#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	#endif
}

__global__ void vis_support_region_kernel(TextureObject<uchar4> colour, TextureObject<uchar4> region, uchar4 bcolour, uchar4 acolour,
		int ox, int oy, int dx, int dy) {
    const int x = blockIdx.x*blockDim.x + threadIdx.x;
    const int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < 0 || y < 0 || x >= colour.width() || y >= colour.height()) return;
	
	// Grid pattern
	if (x % dx != ox || y % dy != oy) return;

	uchar4 base = region.tex2D(x,y);

	// Edge pattern
	//if (base.x != 1) return;
	
	for (int v=-base.z; v<=base.w; ++v) {
		uchar4 baseY = region.tex2D(x,y+v);

		for (int u=-baseY.x; u<=baseY.y; ++u) {
			if (x+u < 0 || y+v < 0 || x+u >= colour.width() || y+v >= colour.height()) continue;
			auto col = colour.tex2D(float(x+u)+0.5f, float(y+v)+0.5f);
			colour(x+u, y+v) = (u==0 || v == 0) ?
					make_uchar4(max(bcolour.x, (unsigned char)col.x), max(bcolour.y, (unsigned char)col.y), max(bcolour.z, (unsigned char)col.z), 0) :
					make_uchar4(max(acolour.x, (unsigned char)col.x), max(acolour.y, (unsigned char)col.y), max(acolour.z, (unsigned char)col.z), 0);
		}
	}
}

void ftl::cuda::vis_support_region(
        ftl::cuda::TextureObject<uchar4> &colour,
		ftl::cuda::TextureObject<uchar4> &region,
		uchar4 bar_colour,
		uchar4 area_colour,
		int ox, int oy, int dx, int dy,
        cudaStream_t stream) {

    const dim3 gridSize((region.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (region.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    vis_support_region_kernel<<<gridSize, blockSize, 0, stream>>>(
		colour,
		region,
		bar_colour,
		area_colour,
		ox,oy,dx,dy
	);
    cudaSafeCall( cudaGetLastError() );


    #ifdef _DEBUG
    cudaSafeCall(cudaDeviceSynchronize());
    #endif
}

// ===== Vis bad edges =========================================================

__global__ void vis_bad_region_kernel(
		TextureObject<uchar4> colour,
		TextureObject<float> depth,
		TextureObject<uchar4> region,
		TextureObject<uchar4> dregion) {
    const int x = blockIdx.x*blockDim.x + threadIdx.x;
    const int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < 0 || y < 0 || x >= colour.width() || y >= colour.height()) return;
	
	// Grid pattern
	//if (x % 50 != 0 || y % 50 != 0) return;

	uchar4 base = region.tex2D(x,y);
	uchar4 baseD = dregion.tex2D(x,y);
	auto col = colour.tex2D((float)x+0.5f,(float)y+0.5f);
	float d = depth.tex2D(x,y);

	if (baseD.x > base.x && baseD.y < base.y) {
		uchar4 baseR = region.tex2D(x+baseD.y+1, y);
		float dR = depth.tex2D(x+baseD.y+1, y);
		//if (x+baseD.y+1-baseR.x <= x) {
			if (d > 0.0f && d < 30.0f && (dR <= 0.0f || dR >= 30.0f)) {
				colour(x,y) = make_uchar4(col.x, col.y, 255, 0);
				depth(x,y) = 0.0f;
			}
		//}
	}
}

void ftl::cuda::vis_bad_region(
		ftl::cuda::TextureObject<uchar4> &colour,
		ftl::cuda::TextureObject<float> &depth,
		ftl::cuda::TextureObject<uchar4> &region,
		ftl::cuda::TextureObject<uchar4> &dregion,
        cudaStream_t stream) {

    const dim3 gridSize((region.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (region.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    vis_bad_region_kernel<<<gridSize, blockSize, 0, stream>>>(
		colour,
		depth,
		region,
		dregion
	);
    cudaSafeCall( cudaGetLastError() );


    #ifdef _DEBUG
    cudaSafeCall(cudaDeviceSynchronize());
    #endif
}
