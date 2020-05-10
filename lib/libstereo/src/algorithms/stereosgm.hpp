#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <opencv2/core/cuda/common.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudastereo.hpp>

#include "stereo.hpp"

#include "util_opencv.hpp"
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

template<typename MatchingCostT, typename ParamsT>
struct StereoSgm {
	ParamsT &params;
	MatchingCostT cost;
	Array2D<typename MatchingCostT::Type> cost_min_paths;
	Array2D<typename MatchingCostT::Type> uncertainty;

	PathAggregator<ftl::stereo::aggregations::StandardSGM<typename MatchingCostT::DataType>> aggr;
	WinnerTakesAll<DisparitySpaceImage<typename MatchingCostT::Type>, float> wta;

	StereoSgm(ParamsT &params, int width, int height, int min_disp, int max_disp) :
		params(params),
		cost(width, height, min_disp, max_disp),
		cost_min_paths(width, height),
		uncertainty(width, height)
		{}

	void compute(cv::OutputArray disparity) {

		// cost aggregation
		ftl::stereo::aggregations::StandardSGM<typename MatchingCostT::DataType> func = {cost.data(), cost_min_paths.data(), params.P1, params.P2};
		auto &out = aggr(func, params.paths);

		cudaSafeCall(cudaDeviceSynchronize());
		if (params.debug) { timer_print("Aggregation"); }

		wta(out, params.subpixel, params.lr_consistency);
		cudaSafeCall(cudaDeviceSynchronize());
		if (params.debug) { timer_print("WTA"); }

		// Drory, A., Haubold, C., Avidan, S., & Hamprecht, F. A. (2014).
		// Semi-global matching: A principled derivation in terms of
		// message passing. Lecture Notes in Computer Science (Including Subseries
		// Lecture Notes in Artificial Intelligence and Lecture Notes in
		// Bioinformatics). https://doi.org/10.1007/978-3-319-11752-2_4

		#if USE_GPU
		cv::cuda::GpuMat uncertainty_gpumat = uncertainty.toGpuMat();
		cv::cuda::subtract(wta.min_cost.toGpuMat(), cost_min_paths.toGpuMat(), uncertainty_gpumat);
		cv::cuda::compare(uncertainty_gpumat, params.uniqueness, uncertainty_gpumat, cv::CMP_GT);
		wta.disparity.toGpuMat().setTo(0, uncertainty_gpumat);
		#else
		cv::Mat uncertainty_mat = uncertainty.toMat();
		cv::subtract(wta.min_cost.toMat(), cost_min_paths.toMat(), uncertainty_mat);
		cv::compare(uncertainty_mat, params.uniqueness, uncertainty, cv::CMP_GT);
		wta.disparity.toMat().setTo(0, uncertainty_mat);
		#endif

		median_filter(wta.disparity, disparity);
		if (params.debug) { timer_print("median filter"); }
	}
};
