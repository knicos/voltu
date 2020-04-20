#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/cuda/common.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>

#include "stereo.hpp"

#include "util_opencv.hpp"
#include "costs/mutual_information.hpp"
#include "dsi.hpp"

#include "wta.hpp"
#include "cost_aggregation.hpp"
#include "aggregations/standard_sgm.hpp"
#include "median_filter.hpp"

#include "costs/dual.hpp"
#include "costs/census.hpp"

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

static void variance_mask(cv::InputArray in, cv::OutputArray out, int wsize=3) {
	if (in.isGpuMat()) {
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
		std::cout << im.type() << "\n";
		std::cout << mean.type() << "\n";
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

struct StereoMiSgm2::Impl {
	CensusMatchingCost census;
	MutualInformationMatchingCost mi;
	Array2D<float> variance;
	Array2D<float> variance_r;
	DualCostsWeighted<CensusMatchingCost, MutualInformationMatchingCost> cost;

	Array2D<unsigned short> cost_min;
	Array2D<unsigned short> cost_min_paths;
	Array2D<unsigned short> uncertainty;
	Array2D<float> confidence;
	Array2D<float> disparity_r;
	Array2D<uchar> l;
	Array2D<uchar> r;

	Mat prior; // used only to calculate MI

	PathAggregator<StandardSGM<DualCostsWeighted<CensusMatchingCost, MutualInformationMatchingCost>::DataType>> aggr;
	WinnerTakesAll<DSImage16U,float> wta;

	Impl(int width, int height, int min_disp, int max_disp) :
		census(width, height, min_disp, max_disp),
		mi(width, height, min_disp, max_disp),
		variance(width, height),
		variance_r(width, height),
		cost(width, height, min_disp, max_disp, census, mi, variance, variance_r),
		cost_min(width, height),
		cost_min_paths(width, height),
		uncertainty(width, height),
		confidence(width, height),
		disparity_r(width, height),
		l(width, height), r(width, height) {}
};

StereoMiSgm2::StereoMiSgm2() : impl_(nullptr) {
	impl_ = new Impl(0, 0, 0, 0);
}

void StereoMiSgm2::setPrior(cv::InputArray prior) {
	if (prior.rows() != impl_->cost.height() || prior.cols() != impl_->cost.width()) {
		return;
	}
	if (prior.isGpuMat()) {
		prior.getGpuMat().download(impl_->prior);
	}
	else {
		prior.getMat().copyTo(impl_->prior);
	}
}

#include <opencv2/highgui.hpp>

void StereoMiSgm2::compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity) {
	cudaSetDevice(0);

	if (l.rows() != impl_->cost.height() || r.cols() != impl_->cost.width()) {
		delete impl_; impl_ = nullptr;
		impl_ = new Impl(l.cols(), l.rows(), params.d_min, params.d_max);
	}

	if (impl_->prior.empty() || impl_->prior.size() != l.size()) {
		// if prior disparity is missing, use random values from uniform
		// distribution
		impl_->prior.create(l.rows(), l.cols(), CV_32FC1);
		cv::randu(impl_->prior, params.d_min, params.d_max);
	}

	mat2gray(l, impl_->l);
	mat2gray(r, impl_->r);

	timer_set();

	cv::cuda::GpuMat var_l = impl_->variance.toGpuMat();
	variance_mask(impl_->l.toGpuMat(), var_l, 9);
	cv::cuda::GpuMat var_r = impl_->variance_r.toGpuMat();
	variance_mask(impl_->r.toGpuMat(), var_r, 9);

	cv::cuda::normalize(var_l, var_l, params.alpha, params.beta, cv::NORM_MINMAX, -1);
	cv::cuda::normalize(var_r, var_r, params.alpha, params.beta, cv::NORM_MINMAX, -1);

	if (l.isMat()) {
		impl_->mi.set(l.getMat(), r.getMat(), impl_->prior);
	}
	else if (l.isGpuMat()) {
		Mat l_;
		Mat r_;
		l.getGpuMat().download(l_);
		r.getGpuMat().download(r_);
		impl_->mi.set(l_, r_, impl_->prior);
	}
	impl_->census.set( impl_->l, impl_->r);
	impl_->cost.set();
	if (params.debug) { timer_print("Matching cost"); }

	cudaSafeCall(cudaDeviceSynchronize());

	StandardSGM<DualCostsWeighted<CensusMatchingCost, MutualInformationMatchingCost>::DataType> func = {impl_->cost.data(), impl_->cost_min_paths.data(), params.P1, params.P2};
	auto &out = impl_->aggr(func, AggregationDirections::ALL);  // params.paths

	cudaSafeCall(cudaDeviceSynchronize());
	if (params.debug) { timer_print("Aggregation"); }

	impl_->wta(out, 0);
	cudaSafeCall(cudaDeviceSynchronize());
	if (params.debug) { timer_print("WTA"); }

	median_filter(impl_->wta.disparity, disparity);
	if (params.debug) { timer_print("median filter"); }
}

StereoMiSgm2::~StereoMiSgm2() {
	if (impl_) {
		delete impl_;
		impl_ = nullptr;
	}
}
