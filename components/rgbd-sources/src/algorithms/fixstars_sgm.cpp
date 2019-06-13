/* Copyright 2019 Nicolas Pope */

#include "fixstars_sgm.hpp"
#include <loguru.hpp>
#include <opencv2/cudastereo.hpp>

using ftl::algorithms::FixstarsSGM;
using cv::Mat;

//static ftl::Disparity::Register fixstarssgm("libsgm", FixstarsSGM::create);

FixstarsSGM::FixstarsSGM(nlohmann::json &config) : Disparity(config) {
	ssgm_ = nullptr;
	use_filter_ = value("use_filter", false);
	// note: (max_disp_ << 4) libsgm subpixel accuracy.
	//       What is the impact in the filter? (possible artifacts)
	filter_ = cv::cuda::createDisparityBilateralFilter(max_disp_ << 4, value("filter_radius", 25), value("filter_iter", 1));
}

void FixstarsSGM::compute(const cv::Mat &l, const cv::Mat &r, cv::Mat &disp) {
	Mat left_disp;
	Mat right_disp;

	Mat lbw, rbw;
	cv::cvtColor(l, lbw, cv::COLOR_BGR2GRAY);
	cv::cvtColor(r, rbw, cv::COLOR_BGR2GRAY);

	if (!ssgm_) { // todo: move to constructor
		ssgm_ = new sgm::StereoSGM(l.cols, l.rows, max_disp_, 8, 16,
			sgm::EXECUTE_INOUT_HOST2HOST,
			sgm::StereoSGM::Parameters(10,120,0.95f,true));
	}

	disp = Mat(cv::Size(l.cols, l.rows), CV_16SC1);

	//auto start = std::chrono::high_resolution_clock::now();
	ssgm_->execute(lbw.data, rbw.data, disp.data);
	//std::chrono::duration<double> elapsed =
	//		std::chrono::high_resolution_clock::now() - start;
	//LOG(INFO) << "CUDA sgm in " << elapsed.count() << "s";
	
	// todo: fix libSGM (return float data or provide mask separately)
	// disparity values set to (256 << 5) in libSGM consistency check 
	Mat bad_pixels = (disp == (256 << 5)); 
	
	disp.setTo(0, bad_pixels);
	Mat left_pixels(disp, cv::Rect(0, 0, max_disp_, disp.rows));
	left_pixels = 0;

	if (use_filter_) {
		cv::cuda::GpuMat l_gpu, disp_gpu, disp_gpu_out;
		// parameters need benchmarking, impact of image
		// quick tests show with parameters (max_disp_, 25, 3)
		// roughly 50% in disparity calculation and 50% in filter
		disp_gpu.upload(disp);
		l_gpu.upload(l);
		filter_->apply(disp_gpu, l_gpu, disp_gpu_out);
		disp_gpu_out.download(disp);
	}
	
	disp.convertTo(disp, CV_32F, 1.0f/16.0f);
}

void FixstarsSGM::setMask(Mat &mask) {
	CHECK(mask.type() == CV_8UC1) << "mask type must be CV_8U";
	
	if (!ssgm_) { // todo: move to constructor
		ssgm_ = new sgm::StereoSGM(mask.cols, mask.rows, max_disp_, 8, 16,
			sgm::EXECUTE_INOUT_HOST2HOST,
			sgm::StereoSGM::Parameters(10,120,0.95f,true));
	}
	
	mask_l_ = mask;
	ssgm_->setMask((uint8_t*) mask.data, mask.cols);
}