#include <ftl/rtcensus.hpp>
#include <vector>
#include <tuple>
#include <bitset>
#include <cmath>
#include <glog/logging.h>

using ftl::RTCensus;
using std::vector;
using cv::Mat;
using cv::Point;
using cv::Size;
using std::tuple;
using std::get;
using std::make_tuple;
using std::bitset;

#define XHI(P1,P2) ((P1 <= P2) ? 0 : 1)

static vector<uint64_t> sparse_census_16x16(Mat &arr) {
	vector<uint64_t> result;
	result.resize(arr.cols*arr.rows,0);

	for (size_t v=7; v<arr.rows-7; v++) {
	for (size_t u=7; u<arr.cols-7; u++) {
		uint64_t r = 0;

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

/*static inline uint8_t hamming(uint64_t n1, uint64_t n2) { 
    return bitset<64>(n1^n2).count();
}*/

static vector<uint16_t> dsi_ca(vector<uint64_t> &census_R, vector<uint64_t> &census_L, size_t w, size_t h, size_t d_start, size_t d_stop, int sign=1) {
	// TODO Add asserts
	assert( census_R.size() == w*h);
	assert( census_L.size() == w*h);
	assert( d_stop-d_start > 0 );

	auto ds = d_stop - d_start;
	vector<uint16_t> result(census_R.size()*ds, 0);

	const size_t eu = (sign>0) ? w-2-ds : w-2;


		for (size_t v=2; v<h-2; v++) {
		for (size_t u=(sign>0)?2:ds+2; u<eu; u++) {
			const size_t ix = v*w*ds+u*ds;
			for (int n=-2; n<=2; n++) {
			const auto u_ = u + n;
	
			for (int m=-2; m<=2; m++) {
			
				for (size_t d=0; d<ds; d++) {
				const auto d_ = d * sign;
				//if (u_+d_ < 0 || u_+d_ >= w) continue;
				const auto v_ = (v + m)*w;
				auto r = census_R[u_+v_];
				auto l = census_L[v_+(u_+d_)];
				result[ix+d] += bitset<64>(r^l).count(); //hamming(r,l);
				}
			}
			
			}
		}
		}

	return result;
}

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
			auto p = make_tuple(d_min, dsi[d_min+vwds+uds]);
			auto pl = make_tuple(d_min-1, dsi[d_min-1+vwds+uds]);
			auto pr = make_tuple(d_min+1, dsi[d_min+1+vwds+uds]);

			d_sub = fit_parabola(p,pl,pr);
		}

		result.at<double>(v,u) = d_sub;
	}
	}

	return result;
}

static cv::Mat consistency(cv::Mat &d_sub_r, cv::Mat &d_sub_l) {
	size_t w = d_sub_r.cols;
	size_t h = d_sub_r.rows;
	Mat result = Mat::zeros(Size(w,h), CV_64FC1);
	
	for (size_t v=0; v<h; v++) {
	for (size_t u=0; u<w; u++) {
		auto a = (int)(d_sub_l.at<double>(v,u));
		if (u-a < 0) continue;
		
		auto b = d_sub_r.at<double>(v,u-a);
		
		if (std::abs(a-b) <= 1.0) result.at<double>(v,u) = std::abs((a+b)/2);
		else result.at<double>(v,u) = 0.0;
	}
	}
	
	return result;
}

void RTCensus::disparity(cv::Mat &l, cv::Mat &r, cv::Mat &disp, size_t num_disp, float gamma, float tau) {
	size_t d_min = 0;
	size_t d_max = num_disp;

	auto start = std::chrono::high_resolution_clock::now();
	auto census_R = sparse_census_16x16(r);
	auto census_L = sparse_census_16x16(l);
	std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
	LOG(INFO) << "Census in " << elapsed.count() << "s";

	start = std::chrono::high_resolution_clock::now();
	auto dsi_ca_R = dsi_ca(census_R, census_L, l.cols, l.rows, d_min, d_max);
	auto dsi_ca_L = dsi_ca(census_L, census_R, l.cols, l.rows, d_min, d_max, -1);
	elapsed = std::chrono::high_resolution_clock::now() - start;
	LOG(INFO) << "DSI in " << elapsed.count() << "s";

	auto disp_R = d_sub(dsi_ca_R, l.cols, l.rows, d_max-d_min);
	auto disp_L = d_sub(dsi_ca_L, l.cols, l.rows, d_max-d_min);
	LOG(INFO) << "Disp done";

	disp = consistency(disp_R, disp_L);

	// TODO confidence and texture filtering
}

