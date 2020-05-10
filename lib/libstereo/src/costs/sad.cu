#include "sad.hpp"

#include "../util.hpp"
#include "../util_opencv.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

void calculate_ii(const cv::Mat &in, cv::Mat &out) {
	out.create(in.size(), CV_32SC1);
	if (in.type() != CV_8UC3) { throw std::exception(); }

	for (int y = 0; y < in.rows; y++) {
		int sum = 0;
		for (int x = 0; x < in.cols; x++) {
			auto rgb = in.at<cv::Vec3b>(y,x);
			sum += (rgb[0] + rgb[1] + rgb[2]);
			if (y == 0) {
				out.at<int32_t>(y,x) = sum;
			}
			else {
				out.at<int32_t>(y,x) = sum + out.at<int32_t>(y-1,x);
			}
		}
	}
}

void SADMatchingCost::set(cv::InputArray l, cv::InputArray r) {
	cv::Mat iil;
	cv::Mat iir;
	cv::Mat iml;
	cv::Mat imr;

	if (l.isGpuMat() && r.isGpuMat()) {
		l.getGpuMat().download(iml);
		r.getGpuMat().download(imr);
	}
	else if (l.isMat() && r.isMat()) {
		iml = l.getMat();
		imr = r.getMat();
	}
	calculate_ii(iml, iil);
	calculate_ii(imr, iir);

	l_.create(l.cols(), l.rows());
	r_.create(r.cols(), r.rows());

	#if USE_GPU
	l_.toGpuMat().upload(iil);
	r_.toGpuMat().upload(iir);
	#else
	iil.copyTo(l_.toMat());
	iir.copyTo(r_.toMat());
	#endif

	data().l = l_.data();
	data().r = r_.data();
}
