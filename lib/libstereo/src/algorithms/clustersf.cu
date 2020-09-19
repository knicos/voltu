#include "stereo.hpp"
#include "stereosgm.hpp"
#include "../filters/salient_gradient.hpp"
#include "../filters/focal_cluster.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/cudaarithm.hpp>

struct StereoCSF::Impl {
	Array2D<uchar> l;
	Array2D<uchar> r;
	Array2D<uchar> gl;
	Array2D<uchar> gr;
	Array2D<uchar> temp;
	Bucket1D<short2, 64> buckets_l;
	Bucket2D<ushort, 64> buckets_r;
	Array1D<int> focal;

	Impl(int width, int height) :
		l(width, height), r(width, height),
		gl(width, height), gr(width, height), temp(width, height),
		buckets_l(height), buckets_r(16, height), focal(1024) {}
};

StereoCSF::StereoCSF() : impl_(nullptr) {
	impl_ = new Impl(0, 0);
}

void StereoCSF::compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity) {

	//cudaSetDevice(0);

	if (l.rows() != impl_->l.height || r.cols() != impl_->l.width) {
		delete impl_; impl_ = nullptr;
		impl_ = new Impl(l.cols(), l.rows());
	}

	mat2gray(l, impl_->l);
	mat2gray(r, impl_->r);

	Array2D<float> disp_array(l.cols(), l.rows());
	disp_array.toGpuMat().setTo(cv::Scalar(0.0f));

	Array2D<float> conf_array(l.cols(), l.rows());
	conf_array.toGpuMat().setTo(cv::Scalar(0.0f));
	
	//int fx=1000;
	//int fy=800;
	for (int fx = 200; fx < l.cols()-200; fx += 50) {
	for (int fy = 200; fy < l.rows()-200; fy += 50) {
		short2 focal_pt_est = {short(l.cols()/2), short(l.rows()/2)};
		int focal_radius_est = 100000;
		int focal_radius = 1000;

		for (int i=0; i<3; ++i) {
			SalientGradientGrouped sgr = {focal_pt_est, focal_radius_est, impl_->r.data(), impl_->gr.data(), impl_->temp.data(), impl_->buckets_r.data(), impl_->r.width, impl_->r.height};
			parallel1DWarpSM(sgr, r.rows(), r.cols());

			short2 focal_pt = {short(fx), short(fy)};
			SalientGradient sgl = {focal_pt, focal_radius, impl_->l.data(), impl_->gl.data(), impl_->temp.data(), impl_->buckets_l.data(), impl_->l.width, impl_->l.height};
			parallel1DWarpSM(sgl, l.rows(), l.cols());
			impl_->focal.toGpuMat().setTo(cv::Scalar(0));	

			FocalCluster fc = {focal_pt, impl_->buckets_l.data(), impl_->buckets_r.data(), impl_->focal.data(), 1024};
			parallel1DWarp(fc, l.rows(), 1);

			cv::Point max_disp;
			cv::cuda::minMaxLoc(impl_->focal.toGpuMat(), nullptr, nullptr, nullptr, &max_disp);

			FocalSelector fs = {focal_pt, int(max_disp.x), impl_->buckets_l.data(), impl_->buckets_r.data(), impl_->focal.data(), disp_array.data(), conf_array.data(), 1024};
			parallel1DWarp(fs, l.rows(), 1);

			// Update right focal point estimate and radius
			focal_pt_est.y = focal_pt.y;
			focal_pt_est.x = focal_pt.x-max_disp.x;
			focal_radius_est = focal_radius;
			focal_radius /= 2;
			//std::cout << "Focal disparity = " << max_disp.x << std::endl;
		}
	}
	}

	disp_array.toGpuMat().download(disparity);

	cv::Mat gradtmp;
	conf_array.toGpuMat().download(gradtmp);
	gradtmp.convertTo(gradtmp, CV_8UC1, 255.0);
	cv::applyColorMap(gradtmp, gradtmp, cv::COLORMAP_INFERNO);
	cv::resize(gradtmp,gradtmp, cv::Size(gradtmp.cols/2, gradtmp.rows/2));
	cv::imshow("Confidence", gradtmp);

	cv::Mat tmp;
	impl_->focal.toGpuMat().download(tmp);

	double minval;
	double maxval;
	int minloc[2];
	int maxloc[2];
	cv::minMaxIdx(tmp, &minval, &maxval, minloc, maxloc);

	std::cout << "Focal Disparity = " << maxloc[1] << std::endl;

	tmp.convertTo(tmp, CV_8UC1, 255.0/maxval);
	cv::resize(tmp,tmp, cv::Size(tmp.cols, 100));

	//#if OPENCV_VERSION >= 40102
	//cv::applyColorMap(tmp, tmp, cv::COLORMAP_TURBO);
	//#else
	cv::applyColorMap(tmp, tmp, cv::COLORMAP_INFERNO);
	//#endif
	cv::imshow("Disparity Hist", tmp);

	cv::Mat imgleft, imgright;
	impl_->l.toGpuMat().download(imgleft);
	impl_->r.toGpuMat().download(imgright);

	cv::cvtColor(imgleft,imgleft, cv::COLOR_GRAY2BGR);
	cv::cvtColor(imgright,imgright, cv::COLOR_GRAY2BGR);
	cv::drawMarker(imgleft, cv::Point(1000,800), cv::Scalar(0,0,255));
	cv::drawMarker(imgright, cv::Point(1000-maxloc[1],800), cv::Scalar(0,0,255));

	cv::resize(imgleft,imgleft, cv::Size(imgleft.cols/2, imgleft.rows/2));
	cv::resize(imgright,imgright, cv::Size(imgright.cols/2, imgright.rows/2));
	cv::imshow("Left Focal", imgleft);
	cv::imshow("Right Focal", imgright);

	//impl_->gr.toGpuMat().download(tmp);
	//cv::resize(tmp,tmp, cv::Size(tmp.cols/2, tmp.rows/2));
	//cv::imshow("Gradients Right", tmp);

	cudaSafeCall(cudaDeviceSynchronize());
	disparity.create(l.rows(), l.cols(), CV_32F);
	cv::waitKey(10000);
}

StereoCSF::~StereoCSF() {
	if (impl_) {
		delete impl_;
		impl_ = nullptr;
	}
}

