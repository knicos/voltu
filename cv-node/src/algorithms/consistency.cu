#include <ftl/cuda_common.hpp>

#define CONSISTENCY_THRESHOLD	1.0f

/*
 * Check for consistency between the LR and RL disparity maps, only selecting
 * those that are similar. Otherwise it sets the disparity to NAN.
 */
__global__ void consistency_kernel(cudaTextureObject_t d_sub_l,
		cudaTextureObject_t d_sub_r, float *disp, int w, int h, int pitch) {

	// TODO This doesn't work at either edge (+-max_disparities)
	for (STRIDE_Y(v,h)) {
	for (STRIDE_X(u,w)) {
		float a = (int)tex2D<float>(d_sub_l, u, v);
		if (u-a < 0) {
			disp[v*pitch+u] = NAN; // TODO Check
			continue;
		}
		
		auto b = tex2D<float>(d_sub_r, u-a, v);
		
		if (abs(a-b) <= CONSISTENCY_THRESHOLD) disp[v*pitch+u] = abs((a+b)/2);
		else disp[v*pitch+u] = NAN;
	}
	}

}

namespace ftl {
namespace cuda {
	void consistency(const TextureObject<float> &dl, const TextureObject<float> &dr,
			TextureObject<float> &disp) {
		dim3 grid(1,1,1);
    	dim3 threads(128, 1, 1);
    	grid.x = cv::cuda::device::divUp(disp.width(), 128);
		grid.y = cv::cuda::device::divUp(disp.height(), 11);
		consistency_kernel<<<grid, threads>>>(
			dl.cudaTexture(),
			dr.cudaTexture(),
			disp.devicePtr(),
			disp.width(),
			disp.height(),
			disp.pitch() / sizeof(float));
		cudaSafeCall( cudaGetLastError() );
	}
}
}

