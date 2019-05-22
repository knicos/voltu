#include "nick.hpp"
#include <vector_types.h>

using ftl::algorithms::NickCuda;
using namespace cv;
using namespace cv::cuda;

static ftl::Disparity::Register nickcuda("nick", NickCuda::create);

NickCuda::NickCuda(nlohmann::json &config) : Disparity(config) {

}

namespace ftl { namespace gpu {
void nick1_call(const PtrStepSz<uchar4> &l, const PtrStepSz<uchar4> &r, const PtrStepSz<float> &disp, size_t num_disp);
}}

void NickCuda::compute(const cv::Mat &l, const cv::Mat &r, cv::Mat &disp) {
	if (disp_.empty()) disp_ = cuda::GpuMat(l.size(), CV_32FC1);
	if (left_.empty()) left_ = cuda::GpuMat(l.size(), CV_8UC4);
	if (right_.empty()) right_ = cuda::GpuMat(l.size(), CV_8UC4);
	
	Mat lhsv, rhsv;
	cv::cvtColor(l, lhsv,  COLOR_BGR2HSV);
	cv::cvtColor(r, rhsv, COLOR_BGR2HSV);
	int from_to[] = {0,0,1,1,2,2,-1,3};
	Mat hsval(lhsv.size(), CV_8UC4);
	Mat hsvar(rhsv.size(), CV_8UC4);
	mixChannels(&lhsv, 1, &hsval, 1, from_to, 4);
	mixChannels(&rhsv, 1, &hsvar, 1, from_to, 4);
	
	left_.upload(hsval);
	right_.upload(hsvar);
	
	ftl::gpu::nick1_call(left_, right_, disp_, 200);
	
	disp_.download(disp);
}



