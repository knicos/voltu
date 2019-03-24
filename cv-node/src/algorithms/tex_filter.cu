#include <ftl/cuda_common.hpp>

#define FILTER_WINDOW 11
#define FILTER_WINDOW_R	5

__global__ void texture_filter_kernel(cudaTextureObject_t t, cudaTextureObject_t d,
		ftl::cuda::TextureObject<float> f, int num_disp, int thresh) { // Thresh = -5000000

	float disp = tex2D<float>(d,u,v);
	int neigh_sq = 0;
	int neigh_sum = 0;

	for (STRIDE_Y(v,h)) {
	for (STRIDE_X(u,w)) {
		for (int m=-FILTER_WINDOW_R; m<=FILTER_WINDOW_R; m++) {
			for (int n=-FILTER_WINDOW_R; n<=FILTER_WINDOW_R; n++) {
				uchar4 neigh = tex2D<uchar4>(t, u+n, v+m);
				neigh_sq += neigh*neigh;
				neigh_sum += neigh;
			}	
		}
	}
	}

	// Texture map filtering
	int tm = (neigh_sq / (FILTER_WINDOW*FILTER_WINDOW)) -
			((neigh_sum*neigh_sum) / (FILTER_WINDOW*FILTER_WINDOW));

	if (tm < thesh) {
		f(u,v) = disp;
	} else {
		f(u,v) = NAN;
	}
}

namespace ftl {
namespace cuda {
	void texture_filter(const TextureObject<uchar4> &t, const TextureObject<float> &d,
			TextureObject<float> &f, int num_disp, int thresh) {
		dim3 grid(1,1,1);
    	dim3 threads(128, 1, 1);
    	grid.x = cv::cuda::device::divUp(disp.width(), 128);
		grid.y = cv::cuda::device::divUp(disp.height(), 1);
		texture_filter_kernel<<<grid, threads>>>
			t.cudaTexture(),
			d.cudaTexture(),
			f,
			num_disp,
			thresh);
		cudaSafeCall( cudaGetLastError() );
	}
}
}

