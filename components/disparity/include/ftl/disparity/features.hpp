#ifndef _FTL_DISPARITY_FEATURES_HPP_
#define _FTL_DISPARITY_FEATURES_HPP_

#include <cuda_runtime.h>
#include <opencv2/core/cuda.hpp>

namespace ftl {
namespace disparity {

class ColourFeatures {
	public:
	ColourFeatures();
	~ColourFeatures();

	enum class Feature {
		ALL = 0,
		RED = 0x01,
		GREEN = 0x02,
		BLUE = 0x04,
		RED_GREEN = 0x03,
		RED_BLUE = 0x05,
		BLUE_GREEN = 0x06,
		WHITE = 0x07
	};

	void generate(
		const cv::cuda::GpuMat &image,
		cudaStream_t
	);

	void visualise(
		Feature f,
		int threshold,
		cv::cuda::GpuMat &out,
		cudaStream_t
	);

	private:
	cv::cuda::GpuMat hls_;
	cv::cuda::GpuMat sig_;
	cv::cuda::GpuMat category_;
};

}
}

#endif