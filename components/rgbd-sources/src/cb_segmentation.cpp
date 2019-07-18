#include "ftl/cb_segmentation.hpp"

#include<algorithm>
#include <math.h>

using cv::Mat;
using cv::Vec3f, cv::Vec4f;

using std::vector;
using std::min;
using std::max;
using std::pair;

using namespace ftl;

CBSegmentation::Pixel::Pixel(const int &index, const uchar *bgr, const int &depth, const long &time) {
    idx = index;
	r = bgr[2];
	g = bgr[1];
	b = bgr[0];
	i = sqrt(r*r + g*g + b*b);
	d = depth;
	t = time;
}


void CBSegmentation::Codeword::set(CBSegmentation::Pixel &px) {
	r = px.r;
	g = px.g;
	b = px.b;
	i_min = px.i;
	i_max = px.i;
	f = px.t;
	lambda = px.t - 1;
	p = px.t;
	q = px.t;
	
	d_m = px.d;
	d_S = 0.0;
	d_f = 1.0;
}

void CBSegmentation::Codeword::update(CBSegmentation::Pixel &px) {
	r = (f * r + px.r) / (f + 1);
	g = (f * g + px.g) / (f + 1);
	b = (f * b + px.b) / (f + 1);
	i_min = min(px.i, i_min);
	i_max = max(px.i, i_max);
	f = f + 1;
	lambda = max(lambda, px.t - q);
	q = px.t;

	if (false /*isValidDepth(px.d)*/) { // check value valid
		float d_prev = d_m;
		d_f = d_f + 1;
		d_m = d_m + (px.d - d_m) / d_f;
		d_S = d_S + (px.d - d_m) * (px.d - d_prev);
	}
}

// eq (2) and BSG
//
bool CBSegmentation::Codeword::colordiff(CBSegmentation::Pixel &px, float epsilon) {
	float x_2 = px.r * px.r + px.g * px.g + px.b * px.b;
	float v_2 = r*r + g*g + b*b;
	float xv_2 = pow(px.r * r + px.g * g + px.b * b, 2);
	float p_2 = xv_2 / v_2;
	return sqrt(x_2 - p_2) < epsilon;
}

// eq (3)
// note: ||x_t|| in the article is equal to I defined in
//       "Algorithm for codebook construction"
//
bool CBSegmentation::Codeword::brightness(CBSegmentation::Pixel &px, float alpha, float beta) {
	return true;
	float i_low = alpha * i_max;
	float i_hi = min(beta * i_max, i_min / alpha);
	return (i_low <= px.i) && (px.i <= i_hi);
}

CBSegmentation::CBSegmentation(
		char codebook_size, size_t width, size_t height,
		float alpha, float beta, float epsilon, float sigma,
		int T_h, int T_add, int T_del) :
		width_(width), height_(height), size_(codebook_size + 1),
		T_h_(T_h), T_add_(T_add), T_del_(T_del),
		alpha_(alpha), beta_(beta), epsilon_(epsilon), sigma_(sigma) {
	
	cb_ = vector<Entry>(width * height * size_);
	for (size_t i = 0; i < cb_.size(); i += size_) {
		cb_[i].size = 0;
	}
}

bool CBSegmentation::processPixel(CBSegmentation::Pixel &px, CBSegmentation::Codeword *codeword) {
	char &size = cb_[size_ * px.idx].size;
	size_t idx_begin = size_ * px.idx + 1;	

	CBSegmentation::Entry::Data *start = &(cb_[idx_begin].data);
	CBSegmentation::Entry::Data *entry = start;

	CBSegmentation::Entry::Data *lru = nullptr;
	CBSegmentation::Entry::Data *lfu = nullptr;
	
	// TODO: benchmark sorting

	// if value is found (in M or H), loop exits early and no further maintenance
	// is done. Maintenance may happen when codeword is not found and all entries
	// are evaluated.
	
	for (int i = 0; i < size; i++) {
		if (entry->type == M) {
			// matching codeword, update and return
			if (entry->cw.brightness(px, alpha_, beta_) && entry->cw.colordiff(px, epsilon_)) {
				entry->cw.update(px);
				codeword = &(entry->cw);
				return true;
			}

			// delete (move last to here) if not accessed for longer time than T_del
			if ((px.t - entry->cw.atime()) > T_del_) {
				size--;
				*entry = *(start + size);
				//std::sort(	cb_.begin() + idx_begin,
				//				cb_.begin() + idx_begin + size,
				//				CompareEntry());
				continue;
			}
			
			// update LFU
			if (!lfu || lfu->cw.freq() > entry->cw.freq()) {
				lfu = entry;
			}
		}
		else if (entry->type == H) {
			// matching codeword, update and return
			if (entry->cw.brightness(px, alpha_, beta_) && entry->cw.colordiff(px, epsilon_)) {
				entry->cw.update(px);

				// should be moved to M? if so, move and return true
				if ((px.t - entry->cw.ctime()) > T_add_) {
					entry->type = M;
					//std::sort(	cb_.begin() + idx_begin,
					//				cb_.begin() + idx_begin + size,
					//				CompareEntry());
					return true;
				}
				else {
					return false;
				}
			}
			
			// delete if lambda lt T_h
			if (entry->cw.getLambda() < T_h_) {
				size--;
				*entry = *(start + size);
				continue;
			}

			// update LRU
			if (!lru || lru->cw.atime() > entry->cw.atime()) {
				lru = entry;
			}
		}
	}

	// not found, create new codeword (to empty position or lru h or lfu m)
	// TODO: Should not prefer H codewords over M codewords?
	if ((size_t)size < (size_ - 1)) {
		entry = start + size;
		size++;
		entry->type = H;
		entry->cw.set(px);
	}
	else if (lru) {
		lru->type = H;
		lru->cw.set(px);
	}
	else {
		lfu->type = H;
		lfu->cw.set(px);
	}

	// sort anyways (frequencies may have changed during earlier iterations)
	//std::sort(cb_.begin() + idx_begin, cb_.begin() + idx_begin + size, CompareEntry());

	return false;
}

void CBSegmentation::apply(Mat &in, Mat &out) {
	if (((size_t)out.rows != height_) || ((size_t)out.cols != width_) 
		|| (out.type() != CV_8UC1) || !out.isContinuous()) {
		out = Mat(height_, width_, CV_8UC1, cv::Scalar(0));
	}
	
	// TODO: thread pool, queue N rows
	#pragma omp parallel for
	for (size_t y = 0; y < height_; ++y) {
		size_t idx = y * width_;
		uchar *ptr_in = in.ptr<uchar>(y);
		uchar *ptr_out = out.ptr<uchar>(y);
		
		for (size_t x = 0; x < width_; ++x, ++idx, ptr_in += 3) {
			auto px = Pixel(idx, ptr_in, 0, t_);
			if(processPixel(px)) {
				ptr_out[x] = 0;
			}
			else {
				ptr_out[x] = 255;
			}
		}
	}

	t_++;
}
