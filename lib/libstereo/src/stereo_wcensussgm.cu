#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <opencv2/core/cuda/common.hpp>
#include <opencv2/cudaarithm.hpp>

#include "stereo.hpp"

#include "util_opencv.hpp"
#include "costs/census.hpp"
#include "dsi.hpp"

#include "wta.hpp"
#include "cost_aggregation.hpp"
#include "aggregations/standard_sgm.hpp"

#include "median_filter.hpp"
#include "dsi_tools.hpp"

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

typedef WeightedCensusMatchingCost MatchingCost;

struct StereoWCensusSgm::Impl {
	MatchingCost cost;
	Array2D<MatchingCost::Type> cost_min_paths;
	Array2D<MatchingCost::Type> uncertainty;
	Array2D<uchar> l;
	Array2D<uchar> r;

	PathAggregator<StandardSGM<MatchingCost::DataType>> aggr;
	WinnerTakesAll<DisparitySpaceImage<MatchingCost::Type>,float> wta;

	Impl(int width, int height, int min_disp, int max_disp) :
		cost(width, height, min_disp, max_disp),
		cost_min_paths(width, height),
		uncertainty(width, height),
		l(width, height), r(width, height)
		{}

};

StereoWCensusSgm::StereoWCensusSgm() : impl_(nullptr) {
	impl_ = new Impl(0, 0, 0, 0);
}

void StereoWCensusSgm::compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity) {
	cudaSetDevice(0);

	if (l.rows() != impl_->cost.height() || r.cols() != impl_->cost.width()) {
		delete impl_; impl_ = nullptr;
		impl_ = new Impl(l.cols(), l.rows(), params.d_min, params.d_max);
	}

	mat2gray(l, impl_->l);
	mat2gray(r, impl_->r);
	timer_set();

	// CT
	impl_->cost.set(impl_->l, impl_->r);

	cudaSafeCall(cudaDeviceSynchronize());
	if (params.debug) { timer_print("census transform"); }

	// cost aggregation
	StandardSGM<MatchingCost::DataType> func = {impl_->cost.data(), impl_->cost_min_paths.data(), params.P1, params.P2};
	auto &out = impl_->aggr(func, params.paths);

	cudaSafeCall(cudaDeviceSynchronize());
	if (params.debug) { timer_print("Aggregation"); }

	impl_->wta(out, params.subpixel, params.lr_consistency);
	cudaSafeCall(cudaDeviceSynchronize());
	if (params.debug) { timer_print("WTA"); }

	// Drory, A., Haubold, C., Avidan, S., & Hamprecht, F. A. (2014).
	// Semi-global matching: A principled derivation in terms of
	// message passing. Lecture Notes in Computer Science (Including Subseries
	// Lecture Notes in Artificial Intelligence and Lecture Notes in
	// Bioinformatics). https://doi.org/10.1007/978-3-319-11752-2_4

	#if USE_GPU
	auto uncertainty = impl_->uncertainty.toGpuMat();
	cv::cuda::subtract(impl_->wta.min_cost.toGpuMat(), impl_->cost_min_paths.toGpuMat(), uncertainty);
	cv::cuda::compare(uncertainty, params.uniqueness, uncertainty, cv::CMP_GT);
	impl_->wta.disparity.toGpuMat().setTo(0, uncertainty);
	#else
	auto uncertainty = impl_->uncertainty.toMat();
	cv::subtract(impl_->wta.min_cost.toMat(), impl_->cost_min_paths.toMat(), uncertainty);
	cv::compare(uncertainty, params.uniqueness, uncertainty, cv::CMP_GT);
	impl_->wta.disparity.toMat().setTo(0, uncertainty);
	#endif

	median_filter(impl_->wta.disparity, disparity);
	if (params.debug) { timer_print("median filter"); }

	Array2D<MatchingCost::Type> dsitmp_dev(l.cols(), l.rows());
	dsi_slice(out, impl_->wta.disparity, dsitmp_dev);
	show_dsi_slice(dsitmp_dev.toGpuMat());
}

StereoWCensusSgm::~StereoWCensusSgm() {
	if (impl_) {
		delete impl_;
		impl_ = nullptr;
	}
}
