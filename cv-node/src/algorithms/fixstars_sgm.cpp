#include <ftl/algorithms/fixstars_sgm.hpp>

using ftl::algorithms::FixstarsSGM;
using namespace cv;

static ftl::Disparity::Register fixstarssgm("libsgm", FixstarsSGM::create);

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
		ssgm_ = new sgm::StereoSGM(l.cols, l.rows, max_disp_, 8, 8, sgm::EXECUTE_INOUT_HOST2HOST);
	}
	
	//disp = Mat();
	
	//if (disp.cols != l.cols || disp.rows != l.rows) {
		disp = Mat(cv::Size(l.cols, l.rows), CV_8UC1);
	//}
	
	ssgm_->execute(lbw.data, rbw.data, disp.data);
	
	disp.convertTo(disp, CV_32F, 1.0f);
}



