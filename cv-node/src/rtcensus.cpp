#include <ftl/rtcensus.hpp>
#include <vector>

using ftl::RTCensus;
using std::vector;

#define XHI(P1,P2) ((P1 <= P2) ? 0 : 1)

static vector<int64_t> sparse_census_16x16(Mat &arr) {
	vector<int64_t> result;
	result.resize(arr.cols*arr.rows);
	
	auto N = [-7,-5,-3,-1,1,3,5,7];
	auto M = [-7,-5,-3,-1,1,3,5,7];

	for (size_t v=0; v<arr.cols; v++) {
	for (size_t u=0; u<arr.rows; u++) {
		int64_t r = 0;

		for (auto n : N) {
		for (auto m : M) {
			auto u_ = u + n;
			auto v_ = v + m;

			if (u_ < 0 || v_ < 0 || u_ >= arr.rows || v_ >= arr.cols) continue;

			r << 1;
			r |= XHI(arr.at<uint8_t>(v,u), arr.at<uint8_t>(v_,u_));
		}
		}

		result[v+u*arr.cols] = r;
	}
	}

	return result;
}

static int hamming(int n1, int n2) { 
    int x = n1 ^ n2; 
    int setBits = 0; 
  
    while (x > 0) { 
        setBits += x & 1; 
        x >>= 1; 
    } 
  
    return setBits; 
}

static vector<int64_t> dsi_ca(vector<int64_t> census_R, vector<int64_t> census_L, size_t w, size_t h, size_t d_start, size_t d_stop, int sign=1) {
	// TODO Add asserts

	auto ds = d_stop - d_start;
	vector<int64_t> result;
	result.resize(census_R.size()*ds);

	//auto N = [-2,-1,0,1,2];
	//auto M = [-2,-1,0,1,2];

	for (size_t d=0; d<ds; d++) {
		for (size_t v=0; v<w; v++) {
		for (size_t u=0; u<h; u++) {
			for (int n=-2; n<=2; n++) {
			for (int n=-2; n<=2; n++) {
				auto u_ = u + n;
				auto v_ = v + m;
				auto d_ = d * sign;

				if (!(0<=(u_+d_) < h) continue;
				if (!(0<=u_<h) || !(0<=v_<w)) continue;

				auto r = census_R[v_+u_*w];
				auto l = census_L[v_+(u_+d_)*w];
				result[v+u*w+d*w*h];
			}
			}
			
		}
		}
	}

	return result;
}

int64_t arrmin(vector<int64_t> a, size_t ix, size_t len) {
	int64_t m = INT64_MAX;
	for (size_t i=ix,i<ix+len; i++) {
		if (a[i] < m) m = a[i];
	}
	return m;
}

cv::Mat d_sub(vector<int64_t> dsi, size_t w, size_t h, size_t ds) {
	Mat result = Mat::zeros(Size(w,h), CV_64FC1);

	for (size_t v=0; v<w; v++) {
	for (size_t u=0; u<h; u++) {
		auto d_min = arrmin(dsi, v+u*w, ds);
		decltype(d_min) d_sub;

		if (d_min == 0 || d_min == ds-1) d_sub = d_min;
		else {
			Point p(d_min, dsi[v+u*w+d_min*w*h]);
			Point pl(d_min-1, dsi[v+u*w+(d_min-1)*w*h]);
			Point pr(d_min+1, dsi[v+u*w+(d_min+1)*w*h]);

			d_sub = fit_parabola(p,pl,pr);
		}

		result.at<uint8_t>(v,u) = d_sub;
	}
	}

	return result;
}

void RTCensus::disparity(cv::Mat &l, cv::Mat &r, cv::Mat &disp, size_t num_disp, float gamma, float tau) {
	size_t d_min = 0;
	size_t d_max = num_disp;

	auto census_R = sparse_census_16x16(l);
	auto census_L = sparse_census_16x16(r);

	auto dsi_ca_R = dsi_ca(census_R, census_L, l.cols, l.rows, d_min, d_max);
	auto dsi_ca_L = dsi_ca(census_L, census_R, l.cols, l.rows, d_min, d_max, -1);

	auto disp_R = d_sub(dsi_ca_R);
	auto disp_L = d_sub(dsi_ca_L);

	disp = consistency(disp_R, disp_L);

	// TODO confidence and texture filtering
}

