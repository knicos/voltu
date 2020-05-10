#include "stereo.hpp"
#include "stereosgm.hpp"
#include "../costs/census.hpp"
#include "../costs/dual.hpp"
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/highgui.hpp>

typedef MultiCostsWeighted<MiniCensusMatchingCost,3> MatchingCost;

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

struct StereoHierCensusSgm::Impl : public StereoSgm<MatchingCost, StereoHierCensusSgm::Parameters> {
    MiniCensusMatchingCost cost_fine;
    MiniCensusMatchingCost cost_medium;
    MiniCensusMatchingCost cost_coarse;
	Array2D<uchar> l;
    Array2D<uchar> r;
    Array2D<float> var_fine;
    Array2D<float> var_medium;
    Array2D<float> var_coarse;

	Impl(StereoHierCensusSgm::Parameters &params, int width, int height, int dmin, int dmax) :
        StereoSgm(params, width, height, dmin, dmax),
        cost_fine(width, height, dmin, dmax),
        cost_medium(width, height, dmin, dmax),
        cost_coarse(width, height, dmin, dmax),
        l(width, height), r(width, height),
        var_fine(width, height),
        var_medium(width, height),
        var_coarse(width, height) {
            cost.add(0, cost_fine, var_fine);
            cost.add(1, cost_medium, var_medium);
            cost.add(2, cost_coarse, var_coarse);
        }
};

StereoHierCensusSgm::StereoHierCensusSgm() : impl_(nullptr) {
	impl_ = new Impl(params, 0, 0, 0, 0);
}

void StereoHierCensusSgm::compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity) {

	//cudaSetDevice(0);

	if (l.rows() != impl_->cost.height() || r.cols() != impl_->cost.width()) {
		delete impl_; impl_ = nullptr;
		impl_ = new Impl(params, l.cols(), l.rows(), params.d_min, params.d_max);
	}

	mat2gray(l, impl_->l);
	mat2gray(r, impl_->r);
    timer_set();

    static constexpr int DOWNSCALE_MEDIUM = 4;
    static constexpr int DOWNSCALE_COARSE = 6;
    
    Array2D<uchar> medium_l(l.cols()/DOWNSCALE_MEDIUM, l.rows()/DOWNSCALE_MEDIUM);
    Array2D<uchar> medium_r(r.cols()/DOWNSCALE_MEDIUM, r.rows()/DOWNSCALE_MEDIUM);
    Array2D<uchar> coarse_l(l.cols()/DOWNSCALE_COARSE, l.rows()/DOWNSCALE_COARSE);
    Array2D<uchar> coarse_r(r.cols()/DOWNSCALE_COARSE, r.rows()/DOWNSCALE_COARSE);
    cv::cuda::resize(impl_->l.toGpuMat(), medium_l.toGpuMat(), cv::Size(medium_l.width, medium_r.height));
    cv::cuda::resize(impl_->r.toGpuMat(), medium_r.toGpuMat(), cv::Size(medium_r.width, medium_r.height));
    cv::cuda::resize(impl_->l.toGpuMat(), coarse_l.toGpuMat(), cv::Size(coarse_l.width, coarse_l.height));
    cv::cuda::resize(impl_->r.toGpuMat(), coarse_r.toGpuMat(), cv::Size(coarse_r.width, coarse_r.height));

    cv::cuda::GpuMat var_fine = impl_->var_fine.toGpuMat();
    variance_mask(impl_->l.toGpuMat(), var_fine, params.var_window);
    cv::cuda::normalize(var_fine, var_fine, params.alpha, params.beta, cv::NORM_MINMAX, -1);

    cv::cuda::GpuMat var_medium; // = impl_->var_medium.toGpuMat();
    variance_mask(medium_l.toGpuMat(), var_medium, params.var_window);
    cv::cuda::normalize(var_medium, var_medium, params.alpha, params.beta, cv::NORM_MINMAX, -1);
    cv::cuda::resize(var_medium, impl_->var_medium.toGpuMat(), cv::Size(l.cols(), l.rows()));

    cv::cuda::GpuMat var_coarse; // = impl_->var_coarse.toGpuMat();
    variance_mask(coarse_l.toGpuMat(), var_coarse, params.var_window);
    cv::cuda::normalize(var_coarse, var_coarse, params.alpha, params.beta, cv::NORM_MINMAX, -1);
    cv::cuda::resize(var_coarse, impl_->var_coarse.toGpuMat(), cv::Size(l.cols(), l.rows()));

    cv::Mat tmp;
    impl_->var_coarse.toGpuMat().download(tmp);
    cv::imshow("Var", tmp);

	// CT
    impl_->cost_fine.set(impl_->l, impl_->r);
    impl_->cost_medium.set(impl_->l, impl_->r, medium_l, medium_r);
    impl_->cost_coarse.set(impl_->l, impl_->r, coarse_l, coarse_r);
    impl_->cost.set();
    impl_->compute(disparity);
    
    Array2D<ExpandingCensusMatchingCost::Type> dsitmp_dev(l.cols(), l.rows());
	dsi_slice(impl_->cost, impl_->wta.disparity, dsitmp_dev);
	show_dsi_slice(dsitmp_dev.toGpuMat());
}

StereoHierCensusSgm::~StereoHierCensusSgm() {
	if (impl_) {
		delete impl_;
		impl_ = nullptr;
	}
}
