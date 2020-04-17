#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "stereo.hpp"
#include "matching_cost.hpp"
#include "stereo_common.hpp"
#include "dsi.hpp"

#include "cost_aggregation.hpp"

#ifdef __GNUG__

#include <chrono>
#include <iostream>
/*
static std::chrono::time_point<std::chrono::system_clock> start;

static void timer_set() {
		start = std::chrono::high_resolution_clock::now();
}

static void timer_print(const std::string &msg, const bool reset=true) {
	auto stop = std::chrono::high_resolution_clock::now();

	char buf[24];
	snprintf(buf, sizeof(buf), "%5i ms  ",
				(int) std::chrono::duration_cast<std::chrono::milliseconds>(stop-start).count());

	std::cout << buf <<  msg << "\n" << std::flush;
	if (reset) { timer_set(); }
}
*/
static void timer_set() {}
static void timer_print(const std::string &msg, const bool reset=true) {}

#else

static void timer_set() {}
static void timer_print(const std::string &msg, const bool reset=true) {}

#endif

using cv::Mat;
using cv::Size;

static int ct_windows_w = 9;
static int ct_windows_h = 7;

struct StereoCensusSgmP::Impl {
	DisparitySpaceImage<unsigned short> dsi;
	CensusMatchingCost cost;
	Mat cost_min;
	Mat cost_min_paths;
	Mat uncertainty;
	Mat confidence;
	Mat disparity_r;
	Mat prior_disparity;
	Mat l;
	Mat r;

	Mat prior;
	Mat search;

	Impl(int width, int height, int min_disp, int max_disp) :
		dsi(width, height, min_disp, max_disp, ct_windows_w*ct_windows_h),
		cost(width, height, min_disp, max_disp, ct_windows_w, ct_windows_h),
		cost_min(height, width, CV_16UC1),
		cost_min_paths(height, width, CV_16UC1),
		uncertainty(height, width, CV_16UC1),
		confidence(height, width, CV_32FC1),
		disparity_r(height, width, CV_32FC1),
		prior_disparity(height, width, CV_32FC1)
		{}

	void cvtColor(const cv::Mat &iml, const cv::Mat &imr) {
		switch (iml.channels()) {
			case 4:
				cv::cvtColor(iml, l, cv::COLOR_BGRA2GRAY);
				break;

			case 3:
				cv::cvtColor(iml, l, cv::COLOR_BGR2GRAY);
				break;

			case 1:
				l = iml;
				break;

			default:
				throw std::exception();
		}

		switch (imr.channels()) {
			case 4:
				cv::cvtColor(imr, r, cv::COLOR_BGRA2GRAY);
				break;

			case 3:
				cv::cvtColor(imr, r, cv::COLOR_BGR2GRAY);
				break;

			case 1:
				r = imr;
				break;

			default:
				throw std::exception();
		}
	}
};

static void compute_P2(const cv::Mat &prior, cv::Mat &P2) {

}

// prior	CV32_FC1
// out		CV8_UC2
//
// TODO: range could be in depth units to get more meningful bounds
static void compute_search_range(const cv::Mat &prior, cv::Mat &out, int dmin, int dmax, int range) {
	out.create(prior.size(), CV_8UC2);
	out.setTo(0);

	for (int y = 0; y < out.rows; y++) {
		for (int x = 0; x < out.cols; x ++) {
			int d = round(prior.at<float>(y,x));
			auto &v = out.at<cv::Vec2b>(y,x);
			if ((d != 0) && (d >= dmin) && (d <= dmax)) {
				v[0] = std::max(dmin, d-range);
				v[1] = std::min(dmax, d+range);
			}
			else {
				v[0] = dmin;
				v[1] = dmax;
			}
		}
	}
}


/**
 * Compute 2D offset image. SGM sweeping direction set with dx and dy.
 *
 * Scharstein, D., Taniai, T., & Sinha, S. N. (2018). Semi-global stereo
 * matching with surface orientation priors. Proceedings - 2017 International
 * Conference on 3D Vision, 3DV 2017. https://doi.org/10.1109/3DV.2017.00033
 */
static void compute_offset_image(const cv::Mat &prior, cv::Mat &out,
		const int dx, const int dy) {

	if (prior.empty()) { return; }

	out.create(prior.size(), CV_16SC1);
	out.setTo(0);

	int y_start;
	int y_stop;
	int x_start;
	int x_stop;

	if (dy < 0) {
		y_start = -dy;
		y_stop = prior.rows;
	}
	else {
		y_start = 0;
		y_stop = prior.rows - dy;
	}

	if (dx < 0) {
		x_start = -dx;
		x_stop = prior.cols;
	}
	else {
		x_start = 0;
		x_stop = prior.cols - dx;
	}

	for (int y = y_start; y < y_stop; y++) {
		const float *ptr_prior = prior.ptr<float>(y);
		const float *ptr_prior_r = prior.ptr<float>(y+dy);
		short *ptr_out = out.ptr<short>(y);

		for (int x = x_start; x < x_stop; x++) {
			// TODO types (assumes signed ptr_out and floating point ptr_prior)
			if (ptr_prior[x] != 0.0 && ptr_prior_r[x+dx] != 0.0) {
				ptr_out[x] = round(ptr_prior[x] - ptr_prior_r[x+dx]);
			}
		}
	}
}

struct AggregationParameters {
	Mat &prior;		//
	Mat &search;	//	search range for each pixel CV_8UC2
	Mat &min_cost;
	const StereoCensusSgmP::Parameters &params;
};

inline int get_jump(const int y, const int x, const int d, const int dy, const int dx, const cv::Mat &prior) {
	if (prior.empty()) {
		return 0;
	}
	else if (dx == 1 && dy == 0) {
		return prior.at<short>(y, x);
	}
	else if (dx == -1 && dy == 0) {
		if (x == prior.cols - 1) { return 0; }
		return -prior.at<short>(y, x+1);
	}
	else if (dx == 0 && dy == 1) {
		return prior.at<short>(y, x);
	}
	else if (dx == 0 && dy == -1) {
		if (y == prior.rows - 1) { return 0; }
		return -prior.at<short>(y+1, x);
	}
	else if (dx == 1 && dy == 1) {
		return prior.at<short>(y, x);
	}
	else if (dx == -1 && dy == -1) {
		if (y == prior.rows - 1 || x == prior.cols - 1) { return 0; }
		return -prior.at<short>(y+1, x+1);
	}
	else if (dx == 1 && dy == -1) {
		return prior.at<short>(y, x);
	}
	else if (dx == -1 && dy == 1) {
		if (y == prior.rows - 1 || x == 0) { return 0; }
		return -prior.at<short>(y+1, x-1);
	}
}

template<typename T=unsigned short>
inline void aggregate(
		AggregationData<CensusMatchingCost, DisparitySpaceImage<T>, T> &data,
		AggregationParameters &params) {

	auto &previous_cost_min = data.previous_cost_min;
	auto &previous = data.previous;
	auto &updates = data.updates;
	auto &out = data.out;
	const auto &in = data.in;
	const auto &x = data.x;
	const auto &y = data.y;
	const auto &i = data.i;
	const T P1 = params.params.P1;
	const T P2 = params.params.P2;

	T cost_min = in.cost_max;

	//int d_start = params.search.at<cv::Vec2b>(y,x)[0];
	//int d_stop = params.search.at<cv::Vec2b>(y,x)[1];
	int d_start = in.disp_min;
	int d_stop =  in.disp_max;

	for (int d = d_start; d <= d_stop; d++) {
		const int j = get_jump(y, x, d, data.dy, data.dx, params.prior);
		const int d_j = std::max(std::min(d+j, previous.disp_max), previous.disp_min);

		const T L_min =
			std::min<T>(previous(0,i,d_j),
				std::min<T>(T(previous_cost_min + P2),
						std::min<T>(T(previous(0,i,std::min(d_j+1, previous.disp_max)) + P1),
									T(previous(0,i,std::max(d_j-1, previous.disp_min)) + P1))
			)
		);

		T C = in(y,x,d);
		T cost_update = L_min + C - previous_cost_min;

		// stop if close to overflow
		if (cost_update > (std::numeric_limits<T>::max() - T(in.cost_max))) { throw std::exception(); }

		updates(0,i,d) = cost_update;
		cost_min = cost_update * (cost_update < cost_min) + cost_min * (cost_update >= cost_min);
	}

	/*const T update_skipped = params.params.P3;
	for (int d = out.disp_min; d < d_start; d++) {
		previous(0,i,d) = update_skipped;
		out(y,x,d) += update_skipped;
	}*/
	for (int d = d_start; d <= d_stop; d++) {
		previous(0,i,d) = updates(0,i,d);
		out(y,x,d) += updates(0,i,d);
	}
	/*for (int d = d_stop+1; d <= out.disp_max; d++) {
		previous(0,i,d) = update_skipped;
		out(y,x,d) += update_skipped;
	}*/

	params.min_cost.at<T>(y, x) = cost_min;
	previous_cost_min = cost_min;
}

StereoCensusSgmP::StereoCensusSgmP() : impl_(nullptr) {
	impl_ = new Impl(0, 0, 0, 0);
}

void StereoCensusSgmP::setPrior(const cv::Mat &prior) {
	prior.copyTo(impl_->prior_disparity);
}

void StereoCensusSgmP::compute(const cv::Mat &l, const cv::Mat &r, cv::Mat disparity) {
	impl_->dsi.clear();
	impl_->uncertainty.setTo(0);

	if (l.rows != impl_->dsi.height || r.cols != impl_->dsi.width) {
		Mat prior = impl_->prior_disparity;

		delete impl_; impl_ = nullptr;
		impl_ = new Impl(l.cols, l.rows, params.d_min, params.d_max);

		if (prior.size() == l.size()) {
			impl_->prior_disparity = prior;
		}
	}

	impl_->cvtColor(l, r);

	timer_set();

	// CT
	impl_->cost.setLeft(impl_->l);
	impl_->cost.setRight(impl_->r);

	if (params.debug) { timer_print("census transform"); }

	// cost aggregation
	AggregationParameters aggr_params = {impl_->prior, impl_->search, impl_->cost_min_paths, params};
	compute_search_range(impl_->prior_disparity, impl_->search, params.d_min, params.d_max, params.range);

	if (params.paths & AggregationDirections::HORIZONTAL) {
		compute_offset_image(impl_->prior_disparity, impl_->prior, 1, 0);
		aggregate_horizontal_all<unsigned short>(
			aggregate<unsigned short>, impl_->cost, impl_->dsi, aggr_params);
		if (params.debug) { timer_print("horizontal aggregation"); }
	}

	if (params.paths & AggregationDirections::VERTICAL) {
		compute_offset_image(impl_->prior_disparity, impl_->prior, 0, 1);
		aggregate_vertical_all<unsigned short>(
			aggregate<unsigned short>, impl_->cost, impl_->dsi, aggr_params);
		if (params.debug) { timer_print("vertical aggregation"); }
	}

	if (params.paths & AggregationDirections::DIAGONAL1) {
		compute_offset_image(impl_->prior_disparity, impl_->prior, 1, 1);
		aggregate_diagonal_upper_all<unsigned short>(
			aggregate<unsigned short>, impl_->cost, impl_->dsi, aggr_params);
		if (params.debug) { timer_print("upper diagonal aggregation"); }
	}

	if (params.paths & AggregationDirections::DIAGONAL2) {
		compute_offset_image(impl_->prior_disparity, impl_->prior, 1, -1);
		aggregate_diagonal_lower_all<unsigned short>(
			aggregate<unsigned short>, impl_->cost, impl_->dsi, aggr_params);
		if (params.debug) { timer_print("lower diagonal aggregation"); }
	}

	if (!(params.paths & AggregationDirections::ALL)) {
		throw std::exception();
	}

	// wta + consistency
	wta(impl_->dsi, disparity, impl_->cost_min, params.subpixel);
	wta_diagonal(impl_->disparity_r, impl_->dsi);
	consistency_check(disparity, impl_->disparity_r);

	// confidence estimate

	// Drory, A., Haubold, C., Avidan, S., & Hamprecht, F. A. (2014).
	// Semi-global matching: A principled derivation in terms of
	// message passing. Lecture Notes in Computer Science (Including Subseries
	// Lecture Notes in Artificial Intelligence and Lecture Notes in
	//Bioinformatics). https://doi.org/10.1007/978-3-319-11752-2_4
	impl_->uncertainty = impl_->cost_min - impl_->cost_min_paths;

	// instead of difference, use ratio
	cv::divide(impl_->cost_min, impl_->cost_min_paths, impl_->confidence, CV_32FC1);

	// confidence threshold
	disparity.setTo(0.0f, impl_->uncertainty > params.uniqueness);

	cv::medianBlur(disparity, disparity, 3);
}

StereoCensusSgmP::~StereoCensusSgmP() {
	if (impl_) {
		delete impl_;
		impl_ = nullptr;
	}
}
