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


#include <ftl/algorithms/rtcensus.hpp>
#include <vector>
#include <tuple>
#include <bitset>
#include <cmath>
#include <chrono>
#include <glog/logging.h>

using ftl::algorithms::RTCensus;
using std::vector;
using cv::Mat;
using cv::Point;
using cv::Size;
using std::tuple;
using std::get;
using std::make_tuple;
using std::bitset;

static ftl::Disparity::Register rtcensus("rtcensus", RTCensus::create);

/* (8) and (16) */
#define XHI(P1,P2) ((P1 <= P2) ? 0 : 1)

/* (14) and (15), based on (9) */
static vector<uint64_t> sparse_census_16x16(const Mat &arr) {
	vector<uint64_t> result;
	result.resize(arr.cols*arr.rows,0);

	/* Loops adapted to avoid edge out-of-bounds checks */
	for (size_t v=7; v<arr.rows-7; v++) {
	for (size_t u=7; u<arr.cols-7; u++) {
		uint64_t r = 0;

		/* 16x16 sparse kernel to 8x8 mask (64 bits) */
		for (int n=-7; n<=7; n+=2) {
		auto u_ = u + n;
		for (int m=-7; m<=7; m+=2) {
			auto v_ = v + m;
			r <<= 1;
			r |= XHI(arr.at<uint8_t>(v,u), arr.at<uint8_t>(v_,u_));
		}
		}

		result[u+v*arr.cols] = r;
	}
	}

	return result;
}

/*
 * (19) note: M and N not the same as in (17), see also (8) in [2].
 * DSI: Disparity Space Image. LTR/RTL matching can be with sign +1/-1
 */
static void dsi_ca(vector<uint16_t> &result, const vector<uint64_t> &census_R, const vector<uint64_t> &census_L, size_t w, size_t h, size_t d_start, size_t d_stop, int sign=1) {
	// TODO Add asserts
	assert( census_R.size() == w*h);
	assert( census_L.size() == w*h);
	assert( d_stop-d_start > 0 );

	auto ds = d_stop - d_start;		// Number of disparities to check
	result.resize(census_R.size()*ds, 0);

	// Change bounds depending upon disparity direction
	const size_t eu = (sign>0) ? w-2-ds : w-2;

	// Adapt bounds to avoid out-of-bounds checks
	for (size_t v=2; v<h-2; v++) {
	for (size_t u=(sign>0)?2:ds+2; u<eu; u++) {
		const size_t ix = v*w*ds+u*ds;

		// 5x5 window size
		for (int n=-2; n<=2; n++) {
		const auto u_ = u + n;
		for (int m=-2; m<=2; m++) {
			const auto v_ = (v + m)*w;
			auto r = census_R[u_+v_];
			
			for (size_t d=0; d<ds; d++) {
				const auto d_ = d * sign;
				auto l = census_L[v_+(u_+d_)];
				result[ix+d] += bitset<64>(r^l).count(); // Hamming distance
			}
		}
		
		}
	}
	}
}

/*
 * Find the minimum value in a sub array.
 * TODO This can be removed entirely (see CUDA version)
 */
static size_t arrmin(vector<uint16_t> &a, size_t ix, size_t len) {
	uint32_t m = UINT32_MAX;
	size_t mi = 0;
	for (size_t i=ix; i<ix+len; i++) {
		if (a[i] < m) {
			m = a[i];
			mi = i;
		}
	}
	return mi-ix;
}

/*
 * WTA + subpixel disparity (parabilic fitting) (20)
 * TODO remove need to pass tuples (see CUDA version)
 */
static inline double fit_parabola(tuple<size_t,uint16_t> p, tuple<size_t,uint16_t> pl, tuple<size_t,uint16_t> pr) {
	double a = get<1>(pr) - get<1>(pl);
	double b = 2 * (2 * get<1>(p) - get<1>(pl) - get<1>(pr));
	return static_cast<double>(get<0>(p)) + (a / b);
}

static cv::Mat d_sub(vector<uint16_t> &dsi, size_t w, size_t h, size_t ds) {
	Mat result = Mat::zeros(Size(w,h), CV_64FC1);
	
	assert( dsi.size() == w*h*ds );

	for (size_t v=0; v<h; v++) {
	const size_t vwds = v * w * ds;
	for (size_t u=0; u<w; u++) {
		const size_t uds = u*ds;
		auto d_min = arrmin(dsi, vwds+uds, ds);
		double d_sub;

		if (d_min == 0 || d_min == ds-1) d_sub = d_min;
		else {
			// TODO Remove use of tuples
			auto p = make_tuple(d_min, dsi[d_min+vwds+uds]);
			auto pl = make_tuple(d_min-1, dsi[d_min-1+vwds+uds]);
			auto pr = make_tuple(d_min+1, dsi[d_min+1+vwds+uds]);

			d_sub = fit_parabola(p,pl,pr);
		}

		result.at<double>(v,u) = d_sub;
	}
	}

	// TODO Parameter pass not return
	return result;
}

/*
 * consistency between LR and RL disparity (23) and (24)
 */
static cv::Mat consistency(cv::Mat &d_sub_r, cv::Mat &d_sub_l) {
	size_t w = d_sub_r.cols;
	size_t h = d_sub_r.rows;
	Mat result = Mat::zeros(Size(w,h), CV_32FC1);
	
	for (size_t v=0; v<h; v++) {
	for (size_t u=0; u<w; u++) {
		auto a = (int)(d_sub_l.at<double>(v,u));
		if (u-a < 0) continue;
		
		auto b = d_sub_r.at<double>(v,u-a);
		
		if (std::abs(a-b) <= 1.0) result.at<float>(v,u) = std::abs((a+b)/2);
		else result.at<float>(v,u) = 0.0f;
	}
	}
	
	return result;
}

RTCensus::RTCensus(nlohmann::json &config)
	:	Disparity(config),
		gamma_(0.0f),
		tau_(0.0f),
		use_cuda_(config.value("use_cuda",true)),
		alternate_(false) {}

/*
 * Choose the implementation and perform disparity calculation.
 */
void RTCensus::compute(const cv::Mat &l, const cv::Mat &r, cv::Mat &disp) {
	#if defined HAVE_CUDA
	if (use_cuda_) {
		computeCUDA(l,r,disp);
	} else {
		computeCPU(l,r,disp);
	}
	#else // !HAVE_CUDA
	computeCPU(l,r,disp);
	#endif
}

void RTCensus::computeCPU(const cv::Mat &l, const cv::Mat &r, cv::Mat &disp) {
	size_t d_min = min_disp_;
	size_t d_max = max_disp_;
	
	Mat lbw, rbw;
	cv::cvtColor(l, lbw,  cv::COLOR_BGR2GRAY);
	cv::cvtColor(r, rbw, cv::COLOR_BGR2GRAY);

	auto start = std::chrono::high_resolution_clock::now();
	auto census_R = sparse_census_16x16(rbw);
	auto census_L = sparse_census_16x16(lbw);
	std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
	LOG(INFO) << "Census in " << elapsed.count() << "s";

	start = std::chrono::high_resolution_clock::now();
	vector<uint16_t> dsi_ca_R,dsi_ca_L;
	dsi_ca(dsi_ca_R,census_R, census_L, l.cols, l.rows, d_min, d_max);
	dsi_ca(dsi_ca_L,census_L, census_R, l.cols, l.rows, d_min, d_max, -1);
	elapsed = std::chrono::high_resolution_clock::now() - start;
	LOG(INFO) << "DSI in " << elapsed.count() << "s";

	auto disp_R = d_sub(dsi_ca_R, l.cols, l.rows, d_max-d_min);
	auto disp_L = d_sub(dsi_ca_L, l.cols, l.rows, d_max-d_min);
	LOG(INFO) << "Disp done";

	disp = consistency(disp_R, disp_L);

	// TODO confidence and texture filtering
}

#if defined HAVE_CUDA

using namespace cv::cuda;
using namespace cv;

#include <vector_types.h>

namespace ftl { namespace gpu {
void rtcensus_call(const PtrStepSz<uchar4> &l, const PtrStepSz<uchar4> &r, const PtrStepSz<float> &disp, size_t num_disp, const int &s=0);
}}

void RTCensus::computeCUDA(const cv::Mat &l, const cv::Mat &r, cv::Mat &disp) {
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
	ftl::gpu::rtcensus_call((alternate_)?left_:left2_, right_, disp_, max_disp_);
	std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
	LOG(INFO) << "CUDA census in " << elapsed.count() << "s";
	
	alternate_ = !alternate_;
	
	// Read disparity from GPU
	disp_.download(disp);
}

#endif

