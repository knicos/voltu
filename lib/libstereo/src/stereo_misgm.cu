#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/cuda/common.hpp>

#include "stereo.hpp"

#include "util_opencv.hpp"
#include "costs/mutual_information.hpp"
#include "dsi.hpp"

#include "wta.hpp"
#include "cost_aggregation.hpp"
#include "aggregations/standard_sgm.hpp"

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

struct StereoMiSgm::Impl {
	MutualInformationMatchingCost cost;

	Array2D<unsigned short> cost_min;
	Array2D<unsigned short> cost_min_paths;
	Array2D<unsigned short> uncertainty;
	Array2D<float> confidence;
	Array2D<float> disparity_r;
	Array2D<uchar> l;
	Array2D<uchar> r;

	Mat prior; // used only to calculate MI

	PathAggregator<StandardSGM<MutualInformationMatchingCost::DataType>> aggr;
	WinnerTakesAll<DSImage16U,float> wta;

	Impl(int width, int height, int min_disp, int max_disp) :
		cost(width, height, min_disp, max_disp),
		cost_min(width, height),
		cost_min_paths(width, height),
		uncertainty(width, height),
		confidence(width, height),
		disparity_r(width, height), l(width, height), r(width, height) {}

};

StereoMiSgm::StereoMiSgm() : impl_(nullptr) {
	impl_ = new Impl(0, 0, 0, 0);
}

void StereoMiSgm::setPrior(cv::InputArray prior) {
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


void StereoMiSgm::compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity) {
	cudaSetDevice(0);

	if (l.rows() != impl_->cost.height() || r.cols() != impl_->cost.width()) {
		delete impl_; impl_ = nullptr;
		impl_ = new Impl(l.cols(), l.rows(), params.d_min, params.d_max);
	}

	//todo prior
	if (impl_->prior.empty() || impl_->prior.size() != l.size()) {
		impl_->prior.create(l.rows(), l.cols(), CV_32FC1);
		cv::randu(impl_->prior, params.d_min, params.d_max);
	}

	impl_->uncertainty.toMat().setTo(0);

	timer_set();

	// CT
	if (l.isMat()) {
		impl_->cost.set(l.getMat(), r.getMat(), impl_->prior);
	}
	else if (l.isGpuMat()) {
		Mat l_;
		Mat r_;
		l.getGpuMat().download(l_);
		r.getGpuMat().download(r_);
		impl_->cost.set(l_, r_, impl_->prior);
	}
	if (params.debug) { timer_print("mutual information"); }

	cudaSafeCall(cudaDeviceSynchronize());
	// cost aggregation
	//AggregationParameters aggr_params = {impl_->cost_min_paths.data(), params};
	StandardSGM<MutualInformationMatchingCost::DataType> func = {impl_->cost.data(), params.P1, params.P2};
	auto &out = impl_->aggr(func, AggregationDirections::ALL);  // params.paths

	cudaSafeCall(cudaDeviceSynchronize());
	if (params.debug) { timer_print("Aggregation"); }

	impl_->wta(out, 0);
	cudaSafeCall(cudaDeviceSynchronize());
	if (params.debug) { timer_print("WTA"); }
	if (disparity.isGpuMat()) {
		impl_->wta.disparity.toGpuMat(disparity.getGpuMatRef());
	}
	else {
		cv::Mat &disparity_ = disparity.getMatRef();
		impl_->wta.disparity.toMat(disparity_);
		cv::medianBlur(disparity_, disparity_, 3);
	}
	// confidence estimate

	// Drory, A., Haubold, C., Avidan, S., & Hamprecht, F. A. (2014).
	// Semi-global matching: A principled derivation in terms of
	// message passing. Lecture Notes in Computer Science (Including Subseries
	// Lecture Notes in Artificial Intelligence and Lecture Notes in
	// Bioinformatics). https://doi.org/10.1007/978-3-319-11752-2_4
	//cv::Mat uncertainty;
	//uncertainty = impl_->cost_min.toMat() - impl_->cost_min_paths.toMat();
	// confidence threshold
	// TODO: estimate confidence from uncertainty and plot ROC curve.
	//disparity.setTo(0.0f, uncertainty > params.uniqueness);
}

StereoMiSgm::~StereoMiSgm() {
	if (impl_) {
		delete impl_;
		impl_ = nullptr;
	}
}
