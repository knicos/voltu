#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <opencv2/core/cuda/common.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/highgui.hpp>

#include "stereo.hpp"

#include "util_opencv.hpp"
#include "costs/census.hpp"
#include "dsi.hpp"

#include "wta.hpp"
#include "cost_aggregation.hpp"
#include "aggregations/adaptive_penalty.hpp"

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
using ftl::stereo::aggregations::AdaptivePenaltySGM;

struct StereoCensusAdaptive::Impl {
	//DisparitySpaceImage<unsigned short> dsi;
	CensusMatchingCost cost;
	Array2D<unsigned short> cost_min_paths;
	Array2D<unsigned short> uncertainty;
	Array2D<float> disparity_r;
	Array2D<uchar> l;
	Array2D<uchar> r;
	Array2D<unsigned short> penalty;

	PathAggregator<AdaptivePenaltySGM<CensusMatchingCost::DataType>> aggr;
	WinnerTakesAll<DSImage16U,float> wta;

	Impl(int width, int height, int min_disp, int max_disp) :
		cost(width, height, min_disp, max_disp),
		cost_min_paths(width, height),
		uncertainty(width, height),
		disparity_r(width, height), l(width, height), r(width, height),
		penalty(width, height) {}

};

StereoCensusAdaptive::StereoCensusAdaptive() : impl_(nullptr) {
	impl_ = new Impl(0, 0, 0, 0);
}

void StereoCensusAdaptive::compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity) {
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

	impl_->penalty.toGpuMat().setTo(params.P1*4);
	auto canny = cv::cuda::createCannyEdgeDetector(30,120);
	cv::cuda::GpuMat edges;
	canny->detect(impl_->l.toGpuMat(), edges);
	impl_->penalty.toGpuMat().setTo(int(float(params.P1)*1.5f), edges);

	cv::Mat penalties;
	impl_->penalty.toGpuMat().download(penalties);
	cv::imshow("Penalties", penalties);

	// cost aggregation
	AdaptivePenaltySGM<CensusMatchingCost::DataType> func = {impl_->cost.data(), impl_->cost_min_paths.data(), params.P1};
	impl_->aggr.getDirectionData(AggregationDirections::LEFTRIGHT).penalties = impl_->penalty;
	impl_->aggr.getDirectionData(AggregationDirections::RIGHTLEFT).penalties = impl_->penalty;
	impl_->aggr.getDirectionData(AggregationDirections::UPDOWN).penalties = impl_->penalty;
	impl_->aggr.getDirectionData(AggregationDirections::DOWNUP).penalties = impl_->penalty;
	impl_->aggr.getDirectionData(AggregationDirections::TOPLEFTBOTTOMRIGHT).penalties = impl_->penalty;
	impl_->aggr.getDirectionData(AggregationDirections::BOTTOMRIGHTTOPLEFT).penalties = impl_->penalty;
	impl_->aggr.getDirectionData(AggregationDirections::BOTTOMLEFTTOPRIGHT).penalties = impl_->penalty;
	impl_->aggr.getDirectionData(AggregationDirections::TOPRIGHTBOTTOMLEFT).penalties = impl_->penalty;
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
		impl_->wta.disparity.toGpuMat().setTo(0, uncertainty);
	}

	median_filter(impl_->wta.disparity, disparity);
	if (params.debug) { timer_print("median filter"); }
}

StereoCensusAdaptive::~StereoCensusAdaptive() {
	if (impl_) {
		delete impl_;
		impl_ = nullptr;
	}
}
