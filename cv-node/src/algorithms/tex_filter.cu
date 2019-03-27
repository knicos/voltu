#include <ftl/cuda_common.hpp>

#define FILTER_WINDOW 11.0
#define FILTER_WINDOW_R	5

// --- Filter by texture complexity --------------------------------------------

__global__ void texture_filter_kernel(cudaTextureObject_t t, cudaTextureObject_t d,
		ftl::cuda::TextureObject<float> f, int num_disp, double thresh) { // Thresh = -5000000

	for (STRIDE_Y(v,f.height())) {
	for (STRIDE_X(u,f.width())) {
		float disp = tex2D<float>(d,u,v);
		double neigh_sq = 0.0;
		double neigh_sum = 0.0;

		for (int m=-FILTER_WINDOW_R; m<=FILTER_WINDOW_R; m++) {
			for (int n=-FILTER_WINDOW_R; n<=FILTER_WINDOW_R; n++) {
				uchar4 neigh = tex2D<uchar4>(t, u+n, v+m);
				neigh_sq += (double)(neigh.z*neigh.z);
				neigh_sum += (double)neigh.z;
			}	
		}

		// Texture map filtering
		double tm = (neigh_sq / (FILTER_WINDOW*FILTER_WINDOW)) -
				((neigh_sum / (FILTER_WINDOW*FILTER_WINDOW)) * (neigh_sum / (FILTER_WINDOW*FILTER_WINDOW)));

		if (tm >= thresh) {
			f(u,v) = disp;
		} else {
			f(u,v) = NAN; //(tm <= 200.0) ? 0 : 200.0f;;
		}
		
		//f(u,v) = tm;
	}
	}
}

namespace ftl {
namespace cuda {
	void texture_filter(const TextureObject<uchar4> &t, const TextureObject<float> &d,
			TextureObject<float> &f, int num_disp, double thresh) {
		dim3 grid(1,1,1);
    	dim3 threads(128, 1, 1);
    	grid.x = cv::cuda::device::divUp(d.width(), 128);
		grid.y = cv::cuda::device::divUp(d.height(), 1);
		texture_filter_kernel<<<grid, threads>>>(
			t.cudaTexture(),
			d.cudaTexture(),
			f,
			num_disp,
			thresh);
		cudaSafeCall( cudaGetLastError() );
	}
}
}

// --- Generate a texture map --------------------------------------------------

__global__ void texture_map_kernel(cudaTextureObject_t t,
		ftl::cuda::TextureObject<float> f) {

	for (STRIDE_Y(v,f.height())) {
	for (STRIDE_X(u,f.width())) {
		double neigh_sq = 0.0;
		double neigh_sum = 0.0;

		for (int m=-FILTER_WINDOW_R; m<=FILTER_WINDOW_R; m++) {
			for (int n=-FILTER_WINDOW_R; n<=FILTER_WINDOW_R; n++) {
				uchar4 neigh = tex2D<uchar4>(t, u+n, v+m);
				neigh_sq += (double)(neigh.z*neigh.z);
				neigh_sum += (double)neigh.z;
			}	
		}

		// Texture map filtering
		double tm = (neigh_sq / (FILTER_WINDOW*FILTER_WINDOW)) -
				((neigh_sum / (FILTER_WINDOW*FILTER_WINDOW)) * (neigh_sum / (FILTER_WINDOW*FILTER_WINDOW)));
		
		f(u,v) = tm;
	}
	}
}

namespace ftl {
namespace cuda {
	void texture_map(const TextureObject<uchar4> &t,
			TextureObject<float> &f) {
		dim3 grid(1,1,1);
    	dim3 threads(128, 1, 1);
    	grid.x = cv::cuda::device::divUp(f.width(), 128);
		grid.y = cv::cuda::device::divUp(f.height(), 1);
		texture_map_kernel<<<grid, threads>>>(
			t.cudaTexture(),
			f);
		cudaSafeCall( cudaGetLastError() );
	}
}
}

