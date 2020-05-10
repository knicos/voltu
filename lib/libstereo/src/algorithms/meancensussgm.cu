#include "stereo.hpp"
#include "stereosgm.hpp"
#include "../costs/census.hpp"
#include <opencv2/cudafilters.hpp>

struct StereoMeanCensusSgm::Impl : public StereoSgm<CensusMatchingCost, StereoMeanCensusSgm::Parameters> {
	Array2D<uchar> l;
    Array2D<uchar> r;
    Array2D<uchar> ml;
    Array2D<uchar> mr;

	Impl(StereoMeanCensusSgm::Parameters &params, int width, int height, int dmin, int dmax) :
        StereoSgm(params, width, height, dmin, dmax), l(width, height), r(width, height),
        ml(width, height), mr(width, height) {}
};

StereoMeanCensusSgm::StereoMeanCensusSgm() : impl_(nullptr) {
	impl_ = new Impl(params, 0, 0, 0, 0);
}

void StereoMeanCensusSgm::compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity) {

	//cudaSetDevice(0);

	if (l.rows() != impl_->cost.height() || r.cols() != impl_->cost.width()) {
		delete impl_; impl_ = nullptr;
		impl_ = new Impl(params, l.cols(), l.rows(), params.d_min, params.d_max);
	}

	mat2gray(l, impl_->l);
	mat2gray(r, impl_->r);
    
    auto filter = cv::cuda::createBoxFilter(CV_8UC1, CV_8UC1, cv::Size(5,5));
    filter->apply(impl_->l.toGpuMat(), impl_->ml.toGpuMat());   // E[X]
    filter->apply(impl_->r.toGpuMat(), impl_->mr.toGpuMat());   // E[X]

    impl_->cost.set(impl_->ml, impl_->mr);

	cudaSafeCall(cudaDeviceSynchronize());
	impl_->compute(disparity);
}

StereoMeanCensusSgm::~StereoMeanCensusSgm() {
	if (impl_) {
		delete impl_;
		impl_ = nullptr;
	}
}
