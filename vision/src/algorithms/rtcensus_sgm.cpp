/* Created by Nicolas Pope and Sebastian Hahta
 *
 * Implementation of algorithm presented in article(s):
 *
 * [1] Humenberger, Engelke, Kubinger: A fast stereo matching algorithm suitable
 *     for embedded real-time systems
 * [2] Humenberger, Zinner, Kubinger: Performance Evaluation of Census-Based
 *     Stereo Matching Algorithm on Embedded and Multi-Core Hardware
 * [3] Humenberger, Engelke, Kubinger: A Census-Based Stereo Vision Algorithm Using Modified Semi-Global Matching
 *     and Plane Fitting to Improve Matching Quality.
 *
 * Equation numbering uses [1] unless otherwise stated
 */


#include <ftl/algorithms/rtcensus_sgm.hpp>
#include <vector>
#include <tuple>
#include <bitset>
#include <cmath>
#include <glog/logging.h>

using ftl::algorithms::RTCensusSGM;
using std::vector;
using cv::Mat;
using cv::Point;
using cv::Size;
using std::tuple;
using std::get;
using std::make_tuple;
using std::bitset;

static ftl::Disparity::Register rtcensus("rtcensus_sgm", RTCensusSGM::create);

RTCensusSGM::RTCensusSGM(nlohmann::json &config)
	:	Disparity(config),
		gamma_(0.0f),
		tau_(0.0f),
		use_cuda_(config.value("use_cuda",true)),
		alternate_(false) {}

/*
 * Choose the implementation and perform disparity calculation.
 */
void RTCensusSGM::compute(const cv::Mat &l, const cv::Mat &r, cv::Mat &disp) {
	#if defined HAVE_CUDA
	if (use_cuda_) {
		computeCUDA(l,r,disp);
	} else {
		//computeCPU(l,r,disp);
		LOG(ERROR) << "RTCensus SGM requires CUDA";
	}
	#else // !HAVE_CUDA
	//computeCPU(l,r,disp);
	LOG(ERROR) << "RTCensus SGM requires CUDA";
	#endif
}

#if defined HAVE_CUDA

using namespace cv::cuda;
using namespace cv;

#include <vector_types.h>

namespace ftl { namespace gpu {
void rtcensus_sgm_call(const PtrStepSz<uchar4> &l, const PtrStepSz<uchar4> &r, const PtrStepSz<float> &disp, size_t num_disp, const int &s=0);
}}

void RTCensusSGM::computeCUDA(const cv::Mat &l, const cv::Mat &r, cv::Mat &disp) {
	// Initialise gpu memory here because we need image size
	if (disp_.empty()) disp_ = cuda::GpuMat(l.size(), CV_32FC1);
	if (left_.empty()) left_ = cuda::GpuMat(l.size(), CV_8UC4);
	if (left2_.empty()) left2_ = cuda::GpuMat(l.size(), CV_8UC4);
	if (right_.empty()) right_ = cuda::GpuMat(l.size(), CV_8UC4);
	
	Mat lhsv, rhsv;
	cv::cvtColor(l, lhsv,  COLOR_BGR2HSV);
	cv::cvtColor(r, rhsv, COLOR_BGR2HSV);
	int from_to[] = {0,0,1,1,2,2,-1,3};
	Mat hsval(lhsv.size(), CV_8UC4);
	Mat hsvar(rhsv.size(), CV_8UC4);
	mixChannels(&lhsv, 1, &hsval, 1, from_to, 4);
	mixChannels(&rhsv, 1, &hsvar, 1, from_to, 4);
	
	// Send images to GPU
	if (alternate_) left_.upload(hsval);
	else left2_.upload(hsval);
	right_.upload(hsvar);

	auto start = std::chrono::high_resolution_clock::now();
	ftl::gpu::rtcensus_sgm_call((alternate_)?left_:left2_, right_, disp_, max_disp_);
	std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
	LOG(INFO) << "CUDA rtcensus_sgm in " << elapsed.count() << "s";
	
	alternate_ = !alternate_;
	
	// Read disparity from GPU
	disp_.download(disp);
}

#endif

