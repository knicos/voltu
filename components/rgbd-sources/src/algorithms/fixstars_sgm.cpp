/* Copyright 2019 Nicolas Pope */

#include "fixstars_sgm.hpp"
#include <glog/logging.h>

using ftl::algorithms::FixstarsSGM;
using cv::Mat;

//static ftl::Disparity::Register fixstarssgm("libsgm", FixstarsSGM::create);

FixstarsSGM::FixstarsSGM(nlohmann::json &config) : Disparity(config) {
	ssgm_ = nullptr;
}

void FixstarsSGM::compute(const cv::Mat &l, const cv::Mat &r, cv::Mat &disp) {
	Mat left_disp;
	Mat right_disp;

	Mat lbw, rbw;
	cv::cvtColor(l, lbw,  cv::COLOR_BGR2GRAY);
	cv::cvtColor(r, rbw, cv::COLOR_BGR2GRAY);

	if (!ssgm_) {
		ssgm_ = new sgm::StereoSGM(l.cols, l.rows, max_disp_, 8, 16,
			sgm::EXECUTE_INOUT_HOST2HOST,
			sgm::StereoSGM::Parameters(10,120,0.95f,true));
	}

	disp = Mat(cv::Size(l.cols, l.rows), CV_16UC1);

	//auto start = std::chrono::high_resolution_clock::now();
	ssgm_->execute(lbw.data, rbw.data, disp.data);
	//std::chrono::duration<double> elapsed =
	//		std::chrono::high_resolution_clock::now() - start;
	//LOG(INFO) << "CUDA sgm in " << elapsed.count() << "s";
	
	// todo: fix libSGM (return float data or provide mask separately)
	// (256 << 5) coded in libSGM consistency check
	Mat bad_pixels = (disp == (256 << 5)); 
	disp.convertTo(disp, CV_32F, 1.0f/16.0f);
	disp.setTo(0, bad_pixels); // decide how bad values should be represented
	
	disp.convertTo(disp, CV_32F, 1.0f/16.0f);
}