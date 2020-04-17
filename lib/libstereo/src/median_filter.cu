#include "median_filter.hpp"
#include "median_filter_fixstars.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/cuda/common.hpp>

void median_filter(cv::InputArray in, cv::OutputArray out) {
	if (in.isGpuMat()) {
		if (in.type() != CV_32F) { throw std::exception(); }

		auto gpuin = in.getGpuMat();
		auto gpuout = out.getGpuMatRef();
		gpuout.create(in.size(), in.type());

		// TODO: 32F median filter
		cv::cuda::GpuMat tmp1(gpuin.size(), CV_16SC1);
		cv::cuda::GpuMat tmp2(gpuin.size(), CV_16SC1);
		gpuin.convertTo(tmp1, CV_16SC1, 16.0, 0.0);

		median_filter((uint16_t*) tmp1.data, (uint16_t*) tmp2.data, tmp1.cols, tmp1.rows, tmp1.step1(), 0);
		cudaStreamSynchronize(0);

		tmp2.convertTo(gpuout, CV_32FC1, 1.0/16.0, 0.0);
	}
	else {
		auto hostin = in.getMat();
		auto hostout = out.getMatRef();
		cv::medianBlur(in, out, 3);
	}
}

void median_filter(Array2D<float> &in, cv::OutputArray out) {
	if (out.isGpuMat()) {
		median_filter(in.toGpuMat(), out);
	}
	else {
		median_filter(in.toMat(), out);
	}
}
