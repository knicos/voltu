#include "segmentation_cuda.hpp"

#define T_PER_BLOCK 8

using ftl::cuda::TextureObject;


__device__ inline int cross(uchar4 p1, uchar4 p2) {
    return max(max(__sad(p1.x,p2.x,0),__sad(p1.y,p2.y,0)), __sad(p1.z,p2.z,0));
}

__device__ uchar4 calculate_support_region(const TextureObject<uchar4> &img, int x, int y, int tau, int v_max, int h_max) {
    int x_min = max(0, x - h_max);
    int x_max = min(img.width()-1, x + h_max);
    int y_min = max(0, y - v_max);
    int y_max = min(img.height()-1, y + v_max);

	uchar4 result = make_uchar4(x - x_min, x_max - x, y - y_min, y_max - y);

    uchar4 colour = img.tex2D(x,y);

    for (int u=x-1; u >= x_min; --u) {
        if (cross(colour, img.tex2D(u,y)) > tau) {
            result.x = x - u;
            break;
        }
    }
    
    for (int u=x+1; u <= x_max; ++u) {
        if (cross(colour, img.tex2D(u,y)) > tau) {
            result.y = u - x;
            break;
        }
    }

    for (int v=y-1; v >= y_min; --v) {
        if (cross(colour, img.tex2D(x,v)) > tau) {
            result.z = y - v;
            break;
        }
    }

    for (int v=y+1; v <= y_max; ++v) {
        if (cross(colour, img.tex2D(x,v)) > tau) {
            result.w = v - y;
            break;
        }
    }

    return result;
}

__global__ void support_region_kernel(TextureObject<uchar4> colour, TextureObject<uchar4> region, int tau, int v_max, int h_max) {
    const int x = blockIdx.x*blockDim.x + threadIdx.x;
    const int y = blockIdx.y*blockDim.y + threadIdx.y;

    if (x < 0 || y < 0 || x >= colour.width() || y >= colour.height()) return;

    region(x,y) = calculate_support_region(colour, x, y, tau, v_max, h_max);
}

void ftl::cuda::support_region(
        ftl::cuda::TextureObject<uchar4> &colour,
        ftl::cuda::TextureObject<uchar4> &region,
        int tau,
        int v_max,
        int h_max,
        cudaStream_t stream) {

    const dim3 gridSize((region.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (region.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    support_region_kernel<<<gridSize, blockSize, 0, stream>>>(colour, region, tau, v_max, h_max);
    cudaSafeCall( cudaGetLastError() );


    #ifdef _DEBUG
    cudaSafeCall(cudaDeviceSynchronize());
    #endif
}

__global__ void vis_support_region_kernel(TextureObject<uchar4> colour, TextureObject<uchar4> region) {
    const int x = blockIdx.x*blockDim.x + threadIdx.x;
    const int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < 0 || y < 0 || x >= colour.width() || y >= colour.height()) return;
	
	// Grid pattern
	if (x % 50 != 0 || y % 50 != 0) return;

	uchar4 base = region.tex2D(x,y);
	
	for (int v=-base.z; v<=base.w; ++v) {
		uchar4 baseY = region.tex2D(x,y+v);

		for (int u=-baseY.x; u<=baseY.y; ++u) {
			if (x+u < 0 || y+v < 0 || x+u >= colour.width() || y+v >= colour.height()) continue;
			uchar4 col = colour.tex2D(x+u, y+v);
			colour(x+u, y+v) = make_uchar4(255,col.y,col.z,0);
		}
	}
}

void ftl::cuda::vis_support_region(
        ftl::cuda::TextureObject<uchar4> &colour,
        ftl::cuda::TextureObject<uchar4> &region,
        cudaStream_t stream) {

    const dim3 gridSize((region.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (region.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    vis_support_region_kernel<<<gridSize, blockSize, 0, stream>>>(colour, region);
    cudaSafeCall( cudaGetLastError() );


    #ifdef _DEBUG
    cudaSafeCall(cudaDeviceSynchronize());
    #endif
}
