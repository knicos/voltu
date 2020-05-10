#include "stable.hpp"
#include "census.hpp"
#include "../util.hpp"
#include "../util_opencv.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

#include <random>
#include <chrono>

#include <cuda_runtime.h>

namespace algorithms {
	template<int NBITS>
	struct Stable {
		__host__ __device__ inline void window(const int y, const int x, uint64_t* __restrict__ out) {
			int32_t accumulator[NBITS] = {0};
			uint16_t i = 0;
			for (int wy = -WINY/2; wy <= WINY/2; wy++) {
				for (int wx = -WINX/2; wx <= WINX/2; wx++) {
					const int16_t value = im(min(height,max(0,int(float(y)*scaleY) + wy)), min(width,max(0,int(float(x)*scaleX) + wx)));
					const int16_t filter = filter_mask(0, i++);
					const int16_t sign = filter > 0 ? 1 : -1;
					// NOTE: indexing starts from 1
					const int16_t index = int(abs(filter)) - 1;
					accumulator[index] += sign*value;
				}
			}

			for (i = 0; i < NBITS;) {
				// zero if first value, otherwise shift to left
				if (i % 64 == 0) { *out = 0; }
				else             { *out = (*out << 1); }
				*out |= ((accumulator[i] > 0) ? 1 : 0);

				i += 1;
				// if all bits set, continue to next element
				if (i % 64 == 0) { out++; }
			}
		}

		__host__ __device__  void operator()(ushort2 thread, ushort2 stride, ushort2 size) {
			for (int y = thread.y+WINY/2; y<size.y-WINY/2-1; y+=stride.y) {
				for (int x = thread.x+WINX/2; x<size.x-WINX/2-1; x+=stride.x) {
					window(y, x, &(out(y, x*WSTEP)));
				}
			}
		}

		const Array2D<uchar>::Data im;
		const Array2D<int16_t>::Data filter_mask;
		Array2D<uint64_t>::Data out;

		const int WINX;
		const int WINY;
		float scaleX;
		float scaleY;
		const int width;
		const int height;

		// number of uint64_t values for each window
		const int WSTEP = (NBITS - 1)/(sizeof(uint64_t)*8) + 1;
	};

}

#include <iostream>
void StableMatchingCost::generateFilterMask(const int wsize, const int bits) {
	if (bits > 127) {
		// TODO: hardcoded in HammingCost template parameters
		throw std::exception();
	}
	cv::Mat mask(cv::Size(wsize*wsize, 1), CV_16SC1, cv::Scalar(0));
	if (!mask.isContinuous()) { throw std::exception(); }

	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator(seed);
	std::uniform_int_distribution<int16_t> distribution(1, bits*2);

	for (int i = 0; i < mask.total(); i++) {
		// index from 1 to bits (instead of 0 to bits-1) value truncated if
		// outside window
		int16_t val = distribution(generator);
		if (val <= bits) {
			mask.at<int16_t>(i) = ((i + val) > mask.total()) ? bits - 1 : val;
		}
		else {
			val = -(val - bits);
			mask.at<int16_t>(i) = ((i + val) < 0) ? 0 : val;
		}
	}

	wsize_ = wsize;
	filter_mask_.create(wsize*wsize, 1);
	filter_mask_.toGpuMat().upload(mask);
}

void StableMatchingCost::set(const Array2D<uchar> &l, const Array2D<uchar> &r) {
	parallel2D<algorithms::Stable<16>>({l.data(), filter_mask_.data(), stable_l_.data(), wsize_, wsize_, 1.0f, 1.0f, l.width, l.height}, l.width, l.height);
	parallel2D<algorithms::Stable<16>>({r.data(), filter_mask_.data(), stable_r_.data(), wsize_, wsize_, 1.0f, 1.0f, r.width, r.height}, r.width, r.height);
}

void StableMatchingCost::set(const Array2D<uchar> &l, const Array2D<uchar> &r, size_t w, size_t h) {
	float scaleX = float(l.width) / float(w);
	float scaleY = float(l.height) / float(h);
	parallel2D<algorithms::Stable<16>>({l.data(), filter_mask_.data(), stable_l_.data(), wsize_, wsize_, scaleX, scaleY, l.width, l.height}, w, h);
	parallel2D<algorithms::Stable<16>>({r.data(), filter_mask_.data(), stable_r_.data(), wsize_, wsize_, scaleX, scaleY, r.width, r.height}, w, h);
}

void StableMatchingCost::set(cv::InputArray l, cv::InputArray r) {
	if (l.type() != CV_8UC1 || r.type() != CV_8UC1) { throw std::exception(); }
	if (l.rows() != r.rows() || l.cols() != r.cols() || l.rows() != height() || l.cols() != width()) {
		throw std::exception();
	}

	if (l.isGpuMat() && r.isGpuMat()) {
		auto ml = l.getGpuMat();
		auto mr = r.getGpuMat();
		set(Array2D<uchar>(ml), Array2D<uchar>(mr));
	}
	else if (l.isMat() && r.isMat()) {
		auto ml = l.getMat();
		auto mr = r.getMat();
		set(Array2D<uchar>(ml), Array2D<uchar>(mr));
	}
	else {
		throw std::exception();
	}
}
