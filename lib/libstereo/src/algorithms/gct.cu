#include "stereo.hpp"
#include "stereosgm.hpp"
#include "../costs/gct.hpp"

struct StereoGCensusSgm::Impl : public StereoSgm<GeneralizedCensusMatchingCost, StereoGCensusSgm::Parameters> {
	Array2D<uchar> l;
	Array2D<uchar> r;

	Impl(StereoGCensusSgm::Parameters &params, int width, int height, int dmin, int dmax) :
		StereoSgm(params, width, height, dmin, dmax), l(width, height), r(width, height) {}
};

StereoGCensusSgm::StereoGCensusSgm() : impl_(nullptr) {
	impl_ = new Impl(params, 0, 0, 0, 0);
}

void StereoGCensusSgm::compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity) {

	if (l.rows() != impl_->cost.height() || r.cols() != impl_->cost.width()) {
		delete impl_; impl_ = nullptr;
		impl_ = new Impl(params, l.cols(), l.rows(), params.d_min, params.d_max);
		impl_->cost.setEdges(pattern_);
	}

	mat2gray(l, impl_->l);
	mat2gray(r, impl_->r);
	impl_->cost.set(impl_->l, impl_->r);

	cudaSafeCall(cudaDeviceSynchronize());
	impl_->compute(disparity);

	median_filter(impl_->wta.disparity, disparity);

	/* without sgm:
	mat2gray(l, impl_->l);
	mat2gray(r, impl_->r);
	impl_->cost.set(impl_->l, impl_->r);

	WinnerTakesAll<GeneralizedCensusMatchingCost> wta;
	wta(impl_->cost, 0, true);

	median_filter(wta.disparity, disparity);
	*/
}

void StereoGCensusSgm::setPattern(StereoGCensusSgm::Pattern pattern, cv::Size size, int param) {
	switch(pattern) {
		case Pattern::DENSE:
			pattern_ = pattern_dense(size);
			break;

		case Pattern::SPARSE:
			pattern_ = pattern_sparse(size, param);
			break;

		case Pattern::RANDOM:
			pattern_ = pattern_random(size, param);
			break;

		case Pattern::GCT:
			pattern_ = pattern_gct(param);
			break;

		default:
			printf("invalid pattern\n");
			throw std::exception();
	}
	impl_->cost.setEdges(pattern_);
}

void StereoGCensusSgm::setPattern(const std::vector<std::pair<cv::Point2i, cv::Point2i>> &edges) {
	pattern_ = edges;
	impl_->cost.setEdges(edges);
}

StereoGCensusSgm::~StereoGCensusSgm() {
	if (impl_) {
		delete impl_;
		impl_ = nullptr;
	}
}
