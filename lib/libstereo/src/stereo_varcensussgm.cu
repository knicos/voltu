#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/cuda/common.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>

#include "stereo.hpp"

#include "util_opencv.hpp"
#include "dsi.hpp"

#include "wta.hpp"
#include "cost_aggregation.hpp"
#include "aggregations/standard_sgm.hpp"
#include "aggregations/adaptive_penalty.hpp"
#include "median_filter.hpp"
#include "dsi_tools.hpp"

#include "costs/census.hpp"
#include "costs/scale.hpp"
#include <chrono>
#include <iostream>

static void timer_set() {}
static void timer_print(const std::string &msg, const bool reset=true) {}

using cv::Mat;
using cv::Size;
using ftl::stereo::aggregations::AdaptivePenaltySGM;

static void variance_mask(cv::InputArray in, cv::OutputArray out, int wsize=3) {
	if (in.isGpuMat() && out.isGpuMat()) {
		cv::cuda::GpuMat im;
		cv::cuda::GpuMat im2;
		cv::cuda::GpuMat mean;
		cv::cuda::GpuMat mean2;

		mean.create(in.size(), CV_32FC1);
		mean2.create(in.size(), CV_32FC1);
		im2.create(in.size(), CV_32FC1);

		if (in.type() != CV_32FC1) {
			in.getGpuMat().convertTo(im, CV_32FC1);
		}
		else {
			im = in.getGpuMat();
		}

		cv::cuda::multiply(im, im, im2);
		auto filter = cv::cuda::createBoxFilter(CV_32FC1, CV_32FC1, cv::Size(wsize,wsize));
		filter->apply(im, mean);   // E[X]
		filter->apply(im2, mean2); // E[X^2]
		cv::cuda::multiply(mean, mean, mean); // (E[X])^2

		// NOTE: floating point accuracy in subtraction
		// (cv::cuda::createBoxFilter only supports 8 bit integer types)
		cv::cuda::subtract(mean2, mean, out.getGpuMatRef()); // E[X^2] - (E[X])^2
	}
	else { throw std::exception(); /* todo CPU version */ }
}

typedef unsigned short CostType;
typedef WeightedCost<CensusMatchingCost, CostType> MatchingCost;

struct StereoVarCensus::Impl {
	CensusMatchingCost census;
	Array2D<float> variance;
	Array2D<float> variance_r;
	MatchingCost cost;

	Array2D<CostType> penalty;
	Array2D<CostType> cost_min;
	Array2D<CostType> cost_min_paths;
	Array2D<CostType> uncertainty;
	Array2D<float> confidence;

	Array2D<uchar> l;
	Array2D<uchar> r;

	Mat prior; // used only to calculate MI

	PathAggregator<AdaptivePenaltySGM<MatchingCost::DataType>> aggr;

	WinnerTakesAll<DSImage16U,float> wta;

	Impl(int width, int height, int min_disp, int max_disp) :
		census(width, height, min_disp, max_disp),
		variance(width, height),
		variance_r(width, height),
		cost(width, height, min_disp, max_disp, census, variance, variance_r),

		penalty(width, height),
		cost_min(width, height),
		cost_min_paths(width, height),
		uncertainty(width, height),
		confidence(width, height),
		l(width, height), r(width, height) {}
};

StereoVarCensus::StereoVarCensus() : impl_(nullptr) {
	impl_ = new Impl(0, 0, 0, 0);
}

void StereoVarCensus::compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity) {
	cudaSetDevice(0);

	if (l.rows() != impl_->cost.height() || r.cols() != impl_->cost.width()) {
		delete impl_; impl_ = nullptr;
		impl_ = new Impl(l.cols(), l.rows(), params.d_min, params.d_max);
	}

	mat2gray(l, impl_->l);
	mat2gray(r, impl_->r);

	timer_set();

	cv::cuda::GpuMat var_l = impl_->variance.toGpuMat();
	variance_mask(impl_->l.toGpuMat(), var_l, params.var_window);
	cv::cuda::GpuMat var_r = impl_->variance_r.toGpuMat();
	variance_mask(impl_->r.toGpuMat(), var_r, params.var_window);

	cv::cuda::normalize(var_l, var_l, params.alpha, params.beta, cv::NORM_MINMAX, -1);
	cv::cuda::normalize(var_r, var_r, params.alpha, params.beta, cv::NORM_MINMAX, -1);

	impl_->census.set(impl_->l, impl_->r);
	impl_->cost.set();

	if (params.debug) { timer_print("Matching cost"); }

	cudaSafeCall(cudaDeviceSynchronize());

	auto penalty = impl_->penalty.toGpuMat();
	penalty.setTo(params.P2);

	AdaptivePenaltySGM<MatchingCost::DataType> func = {impl_->cost.data(), impl_->cost_min_paths.data(), short(params.P1)};

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

	impl_->wta(out, params.subpixel, params.lr_consistency);
	cudaSafeCall(cudaDeviceSynchronize());
	if (params.debug) { timer_print("WTA"); }

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

StereoVarCensus::~StereoVarCensus() {
	if (impl_) {
		delete impl_;
		impl_ = nullptr;
	}
}
