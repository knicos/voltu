/* Copyright 2019 Nicolas Pope */

#include <ftl/algorithms/elas.hpp>
#include <glog/logging.h>

using ftl::algorithms::ELAS;
using cv::Mat;

static ftl::Disparity::Register elass("elas", ELAS::create);

ELAS::ELAS(nlohmann::json &config) : Disparity(config) {
	// TODO(nick) See if these can improve the situation
	param_.postprocess_only_left = true;
	param_.disp_min = 0;
	param_.disp_max = config["maximum"];
	param_.add_corners = 0;
	param_.gamma = 3;
	param_.sradius = 2;
	param_.match_texture = 1;
	param_.ipol_gap_width = 3;
	param_.support_threshold = 0.85;
	elas_ = new Elas(param_);
}

void ELAS::compute(const cv::Mat &l, const cv::Mat &r, cv::Mat &disp) {
	//Mat left_disp;
	//Mat right_disp;

	Mat lbw, rbw;
	cv::cvtColor(l, lbw,  cv::COLOR_BGR2GRAY);
	cv::cvtColor(r, rbw, cv::COLOR_BGR2GRAY);

	disp = Mat(cv::Size(l.cols, l.rows), CV_32F);
	Mat dispr(cv::Size(l.cols, l.rows), CV_32F);
	
	const int32_t dims[3] = {l.cols,l.rows,lbw.step};
	
	if (disp.step/sizeof(float) != lbw.step) LOG(WARNING) << "Incorrect disparity step";

	auto start = std::chrono::high_resolution_clock::now();
	elas_->process(lbw.data, rbw.data, (float*)disp.data, (float*)dispr.data, dims);
	std::chrono::duration<double> elapsed =
			std::chrono::high_resolution_clock::now() - start;
	LOG(INFO) << "Elas in " << elapsed.count() << "s";

	//disp.convertTo(disp, CV_32F, 1.0f);
}

