#include "stereo.hpp"
#include "stereosgm.hpp"
#include "../costs/census.hpp"
#include "costs/scale.hpp"

#include <opencv2/cudafilters.hpp>

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
typedef WeightedCost<ExpandingCensusMatchingCost, CostType> MatchingCost;


struct StereoExCensusSgm::Impl : public StereoSgm<MatchingCost, StereoExCensusSgm::Parameters> {
	Array2D<uchar> l;
    Array2D<uchar> r;
    Array2D<float> variance;
    Array2D<float> variance_r;
    ExpandingCensusMatchingCost excensus;

	Impl(StereoExCensusSgm::Parameters &params, int width, int height, int dmin, int dmax) :
        StereoSgm(params, width, height, dmin, dmax), l(width, height), r(width, height),
        variance(width,height), variance_r(width,height),
        excensus(width, height, dmin, dmax) {
            cost.setCost(excensus);
            cost.setWeights(variance, variance_r);
        }
};

StereoExCensusSgm::StereoExCensusSgm() : impl_(nullptr) {
	impl_ = new Impl(params, 0, 0, 0, 0);
}

void StereoExCensusSgm::compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity) {

	//cudaSetDevice(0);

	if (l.rows() != impl_->cost.height() || r.cols() != impl_->cost.width()) {
		delete impl_; impl_ = nullptr;
		impl_ = new Impl(params, l.cols(), l.rows(), params.d_min, params.d_max);
	}

	mat2gray(l, impl_->l);
    mat2gray(r, impl_->r);
    
    cv::cuda::GpuMat var_l = impl_->variance.toGpuMat();
	variance_mask(impl_->l.toGpuMat(), var_l, params.var_window);
	cv::cuda::GpuMat var_r = impl_->variance_r.toGpuMat();
    variance_mask(impl_->r.toGpuMat(), var_r, params.var_window);
    
    cv::cuda::normalize(var_l, var_l, params.alpha, params.beta, cv::NORM_MINMAX, -1);
	cv::cuda::normalize(var_r, var_r, params.alpha, params.beta, cv::NORM_MINMAX, -1);

	impl_->excensus.setPattern(params.pattern);
    impl_->excensus.set(impl_->l, impl_->r);
    impl_->cost.set();

	cudaSafeCall(cudaDeviceSynchronize());
    impl_->compute(disparity);
    
    Array2D<ExpandingCensusMatchingCost::Type> dsitmp_dev(l.cols(), l.rows());
	dsi_slice(impl_->cost, impl_->wta.disparity, dsitmp_dev);
	show_dsi_slice(dsitmp_dev.toGpuMat());
}

StereoExCensusSgm::~StereoExCensusSgm() {
	if (impl_) {
		delete impl_;
		impl_ = nullptr;
	}
}
