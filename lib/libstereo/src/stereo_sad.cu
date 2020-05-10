#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <opencv2/core/cuda/common.hpp>
#include <opencv2/cudaarithm.hpp>

#include "stereo.hpp"

#include "util_opencv.hpp"
#include "costs/sad.hpp"
#include "dsi.hpp"

#include "wta.hpp"

#include "median_filter.hpp"

#ifdef __GNUG__

#include <chrono>
#include <iostream>

static std::chrono::time_point<std::chrono::system_clock> start;

static void timer_set() {
		start = std::chrono::high_resolution_clock::now();
}

static void timer_print(const std::string &msg, const bool reset=true) {
	auto stop = std::chrono::high_resolution_clock::now();

	char buf[24];
	snprintf(buf, sizeof(buf), "%5i ms  ",
				(int) std::chrono::duration_cast<std::chrono::milliseconds>(stop-start).count());

	std::cout << buf <<  msg << "\n" << std::flush;
	if (reset) { timer_set(); }
}

#else

static void timer_set() {}
static void timer_print(const std::string &msg, const bool reset=true) {}

#endif

using cv::Mat;
using cv::Size;

struct StereoSad::Impl {
	SADMatchingCost cost;
	Array2D<float> disparity_r;

	WinnerTakesAll<SADMatchingCost, float> wta;

	Impl(int width, int height, int min_disp, int max_disp) :
		cost(width, height, min_disp, max_disp),
		disparity_r(width, height) {}

};

StereoSad::StereoSad() : impl_(nullptr) {
	impl_ = new Impl(0, 0, 0, 0);
}

void StereoSad::compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity) {
	cudaSetDevice(0);

	if (l.rows() != impl_->cost.height() || r.cols() != impl_->cost.width()) {
		delete impl_; impl_ = nullptr;
		impl_ = new Impl(l.cols(), l.rows(), params.d_min, params.d_max);
	}

	timer_set();

	impl_->cost.setWindow(params.wsize, params.wsize);
	impl_->cost.set(l, r);

	cudaSafeCall(cudaDeviceSynchronize());

	impl_->wta(impl_->cost, params.subpixel);
	cudaSafeCall(cudaDeviceSynchronize());
	if (params.debug) { timer_print("WTA"); }

	median_filter(impl_->wta.disparity, disparity);
	if (params.debug) { timer_print("median filter"); }
}

StereoSad::~StereoSad() {
	if (impl_) {
		delete impl_;
		impl_ = nullptr;
	}
}
