#pragma once

#include "array2d.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#if USE_GPU
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>

static void mat2gray(const cv::cuda::GpuMat &in, Array2D<unsigned char> &out) {
	if (in.depth() != CV_8U) {
		printf("input must be 8-bit\n");
		throw std::exception();
	}
	if (out.width != in.cols || out.height != in.rows) {
		printf("input and output have different sizes\n");
		throw std::exception();
	}

	switch (in.channels()) {
		case 4:
			cv::cuda::cvtColor(in, out.toGpuMat(), cv::COLOR_BGRA2GRAY);
			break;

		case 3:
			cv::cuda::cvtColor(in, out.toGpuMat(), cv::COLOR_BGR2GRAY);
			break;

		case 1:
			in.copyTo(out.toGpuMat());
			break;

		default:
			printf("bad number of channels\n");
			throw std::exception();
	}
}
#endif

static void mat2gray(const cv::Mat &in, Array2D<unsigned char> &out) {
	if (in.depth() != CV_8U) {
		printf("input must be 8-bit\n");
		throw std::exception();
	}
	if (out.width != in.cols || out.height != in.rows) {
		printf("input and output have different sizes\n");
		throw std::exception();
	}

#ifndef USE_GPU
	switch (in.channels()) {
		case 4:
			cv::cvtColor(in, out.toMat(), cv::COLOR_BGRA2GRAY);
			break;

		case 3:
			cv::cvtColor(in, out.toMat(), cv::COLOR_BGR2GRAY);
			break;

		case 1:
			in.copyTo(out.toMat());
			break;

		default:
			printf("bad number of channels\n");
			throw std::exception();
	}
#else
	cv::Mat tmp;
	switch (in.channels()) {
		case 4:
			cv::cvtColor(in, tmp, cv::COLOR_BGRA2GRAY);
			break;

		case 3:
			cv::cvtColor(in, tmp, cv::COLOR_BGR2GRAY);
			break;

		case 1:
			in.copyTo(tmp);
			break;

		default:
			printf("bad number of channels\n");
			throw std::exception();
	}

	out.toGpuMat().upload(tmp);
#endif
}

static void mat2gray(cv::InputArray in, Array2D<unsigned char> &out) {
	if (in.isGpuMat()) {
		#if USE_GPU
		mat2gray(in.getGpuMat(), out);
		#endif
	}
	else if (in.isMat()) {
		mat2gray(in.getMat(), out);
	}
	else {
		printf("bad input type\n");
		throw std::exception();
	}
}

/*
template<typename T, int channels>
constexpr int cpp2cvtype() {
	static_assert(channels > 0 && channels <= 4, "unsupported number of channels");

	if (std::is_same<T, char>::value) {
		switch(channels) {
			case 1: return CV_8SC1;
			case 2: return CV_8SC2;
			case 3: return CV_8SC3;
			case 4: return CV_8SC4;
		}
	}
	else if (std::is_same<T, unsigned char>::value) {
		switch(channels) {
			case 1: return CV_8SC1;
			case 2: return CV_8UC2;
			case 3: return CV_8UC3;
			case 4: return CV_8UC4;
		}
	}
	else if (std::is_same<T, short>::value) {
		switch(channels) {
			case 1: return CV_16SC1;
			case 2: return CV_16SC2;
			case 3: return CV_16SC3;
			case 4: return CV_16SC4;
		}
	}
	else if (std::is_same<T, unsigned short>::value) {
		switch(channels) {
			case 1: return CV_16UC1;
			case 2: return CV_16UC2;
			case 3: return CV_16UC3;
			case 4: return CV_16UC4;
		}
	}

	else if (std::is_same<T, float>::value) {
		switch(channels) {
			case 1: return CV_32FC1;
			case 2: return CV_32FC2;
			case 3: return CV_32FC3;
			case 4: return CV_32FC4;
		}
	}
	else if (std::is_same<T, double>::value) {
		switch(channels) {
			case 1: return CV_64FC1;
			case 2: return CV_64FC2;
			case 3: return CV_64FC3;
			case 4: return CV_64FC4;
		}
	}
	else {
		// ideally should be compile time error
		throw std::logic_error("no matching OpenCV type");
	}

	return -1;
}*/
