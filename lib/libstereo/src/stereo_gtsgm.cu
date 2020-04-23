#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <opencv2/core/cuda/common.hpp>
#include <opencv2/cudaarithm.hpp>

#include "stereo.hpp"

#include "util_opencv.hpp"
#include "costs/gt.hpp"
#include "dsi.hpp"

#include "wta.hpp"
#include "cost_aggregation.hpp"
#include "aggregations/standard_sgm.hpp"

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
using ftl::stereo::aggregations::StandardSGM;

struct StereoGtSgm::Impl {
	GtMatchingCost cost;
	Array2D<float> cost_min_paths;
	Array2D<float> uncertainty;
	Array2D<float> disparity_r;
	Array2D<uchar> l;
	Array2D<uchar> r;

	PathAggregator<StandardSGM<GtMatchingCost::DataType>> aggr;
	WinnerTakesAll<DSImageFloat,float> wta;

	Impl(int width, int height, int min_disp, int max_disp) :
		cost(width, height, min_disp, max_disp),
		cost_min_paths(width, height),
		uncertainty(width, height),
		disparity_r(width, height), l(width, height), r(width, height) {}

};

StereoGtSgm::StereoGtSgm() : impl_(nullptr) {
	impl_ = new Impl(0, 0, 0, 0);
}

void StereoGtSgm::compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity) {
	cudaSetDevice(0);

	if (l.rows() != impl_->cost.height() || l.cols() != impl_->cost.width()) {
		throw std::exception();
	}

	cudaSafeCall(cudaDeviceSynchronize());
	timer_set();

	// cost aggregation
	StandardSGM<GtMatchingCost::DataType> func = {impl_->cost.data(), impl_->cost_min_paths.data(), params.P1, params.P2};
	auto &out = impl_->aggr(func, params.paths);

	cudaSafeCall(cudaDeviceSynchronize());
	if (params.debug) { timer_print("Aggregation"); }

	impl_->wta(out, 0);
	cudaSafeCall(cudaDeviceSynchronize());
	if (params.debug) { timer_print("WTA"); }

	// Drory, A., Haubold, C., Avidan, S., & Hamprecht, F. A. (2014).
	// Semi-global matching: A principled derivation in terms of
	// message passing. Lecture Notes in Computer Science (Including Subseries
	// Lecture Notes in Artificial Intelligence and Lecture Notes in
	// Bioinformatics). https://doi.org/10.1007/978-3-319-11752-2_4

	if (disparity.isGpuMat()) {
		auto uncertainty = impl_->uncertainty.toGpuMat();
		cv::cuda::subtract(impl_->wta.min_cost.toGpuMat(), impl_->cost_min_paths.toGpuMat(), uncertainty);
		cv::cuda::compare(uncertainty, params.uniqueness, uncertainty, cv::CMP_GT);
		impl_->wta.disparity.toGpuMat().setTo(0, uncertainty);
	}
	else {
		auto uncertainty = impl_->uncertainty.toMat();
		cv::subtract(impl_->wta.min_cost.toMat(), impl_->cost_min_paths.toMat(), uncertainty);
		cv::compare(uncertainty, params.uniqueness, uncertainty, cv::CMP_GT);
		impl_->wta.disparity.toMat().setTo(0, uncertainty);
	}

	median_filter(impl_->wta.disparity, disparity);
	if (params.debug) { timer_print("median filter"); }
}

void StereoGtSgm::setPrior(cv::InputArray disp) {
	if (disp.rows() != impl_->cost.height() || disp.cols() != impl_->cost.width()) {
		delete impl_; impl_ = nullptr;
		impl_ = new Impl(disp.cols(), disp.rows(), params.d_min, params.d_max);
	}
	impl_->cost.set(disp);
}

StereoGtSgm::~StereoGtSgm() {
	if (impl_) {
		delete impl_;
		impl_ = nullptr;
	}
}
