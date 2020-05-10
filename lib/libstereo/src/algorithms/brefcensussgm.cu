#include "stereo.hpp"
#include "stereosgm.hpp"
#include "../costs/census.hpp"
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/highgui.hpp>

struct StereoBRefCensusSgm::Impl : public StereoSgm<CensusMatchingCost, StereoBRefCensusSgm::Parameters> {
	Array2D<uchar> l;
    Array2D<uchar> r;
    Array2D<uchar> bl;
    Array2D<uchar> br;

	Impl(StereoBRefCensusSgm::Parameters &params, int width, int height, int dmin, int dmax) :
        StereoSgm(params, width, height, dmin, dmax), l(width, height), r(width, height),
        bl(width, height), br(width, height) {}
};

StereoBRefCensusSgm::StereoBRefCensusSgm() : impl_(nullptr) {
	impl_ = new Impl(params, 0, 0, 0, 0);
}

void StereoBRefCensusSgm::compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity) {

	//cudaSetDevice(0);

	if (l.rows() != impl_->cost.height() || r.cols() != impl_->cost.width()) {
		delete impl_; impl_ = nullptr;
		impl_ = new Impl(params, l.cols(), l.rows(), params.d_min, params.d_max);
	}

	mat2gray(l, impl_->l);
    mat2gray(r, impl_->r);
    
    cv::cuda::bilateralFilter(impl_->l.toGpuMat(), impl_->bl.toGpuMat(), 5, 50, 100);
    cv::cuda::bilateralFilter(impl_->r.toGpuMat(), impl_->br.toGpuMat(), 5, 50, 100);

	impl_->cost.set(impl_->bl, impl_->br, impl_->l, impl_->r);

	cudaSafeCall(cudaDeviceSynchronize());
	impl_->compute(disparity);
}

StereoBRefCensusSgm::~StereoBRefCensusSgm() {
	if (impl_) {
		delete impl_;
		impl_ = nullptr;
	}
}
