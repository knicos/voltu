#include "stereo.hpp"
#include "stereosgm.hpp"
#include "../costs/tcensus.hpp"

struct StereoTCensusSgm::Impl : public StereoSgm<TCensusMatchingCost, StereoTCensusSgm::Parameters> {
	Array2D<uchar> l;
	Array2D<uchar> r;

	Impl(StereoTCensusSgm::Parameters &params, int width, int height, int dmin, int dmax) :
		StereoSgm(params, width, height, dmin, dmax), l(width, height), r(width, height) {}
};

StereoTCensusSgm::StereoTCensusSgm() : impl_(nullptr) {
	impl_ = new Impl(params, 0, 0, 0, 0);
}

void StereoTCensusSgm::compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity) {

	//cudaSetDevice(0);

	if (l.rows() != impl_->cost.height() || r.cols() != impl_->cost.width()) {
		delete impl_; impl_ = nullptr;
		impl_ = new Impl(params, l.cols(), l.rows(), params.d_min, params.d_max);
	}

	mat2gray(l, impl_->l);
	mat2gray(r, impl_->r);
	timer_set();

	impl_->cost.setT(params.t);
	impl_->cost.set(impl_->l, impl_->r);
	impl_->compute(disparity);
}

StereoTCensusSgm::~StereoTCensusSgm() {
	if (impl_) {
		delete impl_;
		impl_ = nullptr;
	}
}
