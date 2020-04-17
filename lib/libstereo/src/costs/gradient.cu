#include "gradient.hpp"
#include "../util.hpp"
#include "../util_opencv.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

#include <cuda_runtime.h>

void GradientMatchingCostL2::set(const Array2D<uchar>& l, const Array2D<uchar>& r) {
	// todo: size check
	#if USE_GPU
	cv::Mat m;
	cv::Scharr(l.toMat(), m, CV_16S, 1, 0);
	l_dx_.toGpuMat().upload(m);
	cv::Scharr(l.toMat(), m, CV_16S, 0, 1);
	l_dy_.toGpuMat().upload(m);
	cv::Scharr(r.toMat(), m, CV_16S, 1, 0);
	r_dx_.toGpuMat().upload(m);
	cv::Scharr(r.toMat(), m, CV_16S, 0, 1);
	l_dy_.toGpuMat().upload(m);
	#else
	cv::Scharr(l.toMat(), l_dx_.toMat(), CV_16S, 1, 0);
	cv::Scharr(l.toMat(), l_dy_.toMat(), CV_16S, 0, 1);
	cv::Scharr(r.toMat(), r_dx_.toMat(), CV_16S, 1, 0);
	cv::Scharr(r.toMat(), r_dy_.toMat(), CV_16S, 0, 1);
	#endif
}
