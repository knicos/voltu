#include "gct.hpp"
#include "../util.hpp"

#include <random>

static const int NBITS = 128;

namespace algorithms {
	/** Fife, W. S., & Archibald, J. K. (2012). Improved census transforms for
	* resource-optimized stereo vision. IEEE Transactions on Circuits and
	* Systems for Video Technology, 23(1), 60-73.
	*/

	template<int BITS>
	struct GeneralizedCensusTransform {
		static_assert(BITS%64 == 0, "size must be multiple of 64");

		__host__ __device__ inline void compute(const int y, const int x, uint64_t* __restrict__ out) {

			uint8_t i = 0; // bit counter for *out
			// BUG in operator(), gets called more than once per pixel; local
			// variable for sub-bitstring to avoid data race (no read
			// dependency to out; writes are identical)
			uint64_t res = 0;

			for (int e = 0; e < nedges; e++) {

				// edges contain window indices, calculate window coordinates
				//const int y1 = y + edges(e,0) % WINY - WINY/2;
				//const int x1 = x + edges(e,0) / WINY - WINX/2;
				//const int y2 = y + edges(e,1) % WINY - WINY/2;
				//const int x2 = x + edges(e,1) / WINY - WINX/2;

				// edges contain relative pixel coordinates
				const int x1 = x + edges(e,0);
				const int y1 = y + edges(e,1);
				const int x2 = x + edges(e,2);
				const int y2 = y + edges(e,3);

				res = (res << 1);
				res |= ((im(y1,x1) < im(y2,x2)) ? 1 : 0);

				// if all bits set, continue to next element
				if (++i % 64 == 0) {
					*out = res;
					out++;
				}
			}

			// zero remaining bits (less edges than bits in output array)
			for(i = BITS/64 - i/64; i > 0; i--) {
				*out++ = res;
				res = 0;
			}
		}

		__host__ __device__  void operator()(ushort2 thread, ushort2 stride, ushort2 size) {
			for (int y = thread.y+winy/2; y<size.y-winy/2-1; y+=stride.y) {
				for (int x = thread.x+winx/2; x<size.x-winx/2-1; x+=stride.x) {
					compute(y, x, &(out(y, x*WSTEP)));
				}
			}
		}

		int nedges;
		int winx;
		int winy;
		Array2D<char>::Data edges;
		Array2D<uchar>::Data im;
		Array2D<uint64_t>::Data out;

		// number of uint64_t values for each window
		static constexpr int WSTEP = (BITS - 1)/(sizeof(uint64_t)*8) + 1;
	};
}

void GeneralizedCensusMatchingCost::set(const Array2D<uchar> &l, const Array2D<uchar> &r) {
	if (edges_.height == 0) {
		printf("edges must be set before processing input images\n");
		throw std::exception();
	}

	int winx = std::max(std::abs(pmin.x), pmax.x)*2 + 1;
	int winy = std::max(std::abs(pmin.y), pmax.y)*2 + 1;
	parallel2D<algorithms::GeneralizedCensusTransform<128>>({
			edges_.height, winx, winy,
			edges_.data(), l.data(), ct_l_.data()
		}, l.width, l.height);
	parallel2D<algorithms::GeneralizedCensusTransform<128>>({
			edges_.height, winx, winy,
			edges_.data(), r.data(), ct_r_.data()
		}, r.width, r.height);
}

void GeneralizedCensusMatchingCost::setEdges(const std::vector<std::pair<cv::Point2i, cv::Point2i>> &edges) {
	if (edges.size() > NBITS) {
		printf("Too many edges %i, maximum number %i\n", int(edges.size()), NBITS);
		throw std::exception(); // too many edges
	}

	cv::Mat data_(cv::Size(4, edges.size()), CV_8SC1);
	for (size_t i = 0; i < edges.size(); i++) {
		const auto &p1 = edges[i].first;
		const auto &p2 = edges[i].second;

		data_.at<char>(i,0) = p1.x;
		data_.at<char>(i,1) = p1.y;
		data_.at<char>(i,2) = p2.x;
		data_.at<char>(i,3) = p2.y;

		pmax.x = std::max(pmax.x, std::max(p1.x, p2.x));
		pmax.y = std::max(pmax.y, std::max(p1.y, p2.y));
		pmin.x = std::min(pmax.x, std::min(p1.x, p2.x));
		pmin.y = std::min(pmax.y, std::min(p1.y, p2.y));
	}

	edges_.create(4, edges.size());
	#ifdef USE_GPU
	edges_.toGpuMat().upload(data_);
	#else
	data_.copyTo(edges_.toMat());
	#endif

	// normalization factor: 1.0/(number of comparisons)
	data().normalize = 1.0f/float(edges.size());
}


void GeneralizedCensusMatchingCost::set(cv::InputArray l, cv::InputArray r) {
	if (l.type() != CV_8UC1 || r.type() != CV_8UC1) { throw std::exception(); }
	if (l.rows() != r.rows() || l.cols() != r.cols() || l.rows() != height() || l.cols() != width()) {
		throw std::exception();
	}

	if (l.isGpuMat() && r.isGpuMat()) {
		auto ml = l.getGpuMat();
		auto mr = r.getGpuMat();
		set(Array2D<uchar>(ml), Array2D<uchar>(mr));
	}
	else if (l.isMat() && r.isMat()) {
		auto ml = l.getMat();
		auto mr = r.getMat();
		set(Array2D<uchar>(ml), Array2D<uchar>(mr));
	}
	else {
		printf("Bad input array type\n");
		throw std::exception();
	}
}

// ==== Pattern generators =====================================================

std::vector<std::pair<cv::Point2i, cv::Point2i>> pattern_dense(const cv::Size size) {
	return pattern_sparse(size, 1);
}

std::vector<std::pair<cv::Point2i, cv::Point2i>> pattern_sparse(const cv::Size size, int step) {
	std::vector<std::pair<cv::Point2i, cv::Point2i>> pattern;

	for (int y = -size.height/2; y <= size.height/2; y += step) {
		for (int x = -size.width/2; x <= size.width/2; x += step) {
			if (cv::Point2i{x, y} == cv::Point2i{0, 0}) { continue; }
			pattern.push_back({{0, 0}, {x, y}});
		}
	}

	return pattern;
}

std::vector<std::pair<cv::Point2i, cv::Point2i>> pattern_random(const cv::Size size, int nedges) {
	std::vector<std::pair<cv::Point2i, cv::Point2i>> pattern;

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<> rand_x(-size.width/2, size.width/2);
	std::uniform_int_distribution<> rand_y(-size.height/2, size.height/2);

	for (int i = 0; i < nedges; i++) {
		cv::Point2i p1;
		cv::Point2i p2;
		do {
			p1 = {rand_x(gen), rand_y(gen)};
			p2 = {rand_x(gen), rand_y(gen)};
		}
		while (p1 == p2); // try again if points happen to be same

		pattern.push_back({p1, p2});
	}

	return pattern;
}

std::vector<std::pair<cv::Point2i, cv::Point2i>> pattern_random(const cv::Size size) {
	return pattern_random(size, size.width*size.height);
}

std::vector<std::pair<cv::Point2i, cv::Point2i>> pattern_gct(int nedges) {
	std::vector<std::pair<cv::Point2i, cv::Point2i>> pattern;
	pattern.reserve(nedges);
	switch(nedges) {
		case 16:
			pattern.push_back({{-2, -1}, {2, 1}});
			pattern.push_back({{-2, 1}, {2, -1}});
			pattern.push_back({{-1, -2}, {1, 2}});
			pattern.push_back({{1, -2}, {-1, -2}});
			[[fallthrough]]

		case 12:
			pattern.push_back({{-1, -1}, {1, 0}});
			pattern.push_back({{1, -1}, {-1, 0}});
			pattern.push_back({{-1, 1}, {1, 0}});
			pattern.push_back({{1, 1}, {-1, 0}});
			[[fallthrough]]

		case 8:
			pattern.push_back({{-2, -2}, {2, 2}});
			pattern.push_back({{-2, 2}, {2, -2}});
			pattern.push_back({{0, -2}, {0, 2}});
			pattern.push_back({{-2, 0}, {2, 0}});
			[[fallthrough]]

		case 4:
			pattern.push_back({{-1, -1}, {1, 1}});
			pattern.push_back({{-1, 1}, {1, -1}});
			[[fallthrough]]

		case 2:
			pattern.push_back({{0, -1}, {0, 1}});
			[[fallthrough]]

		case 1:
			pattern.push_back({{-1, 0}, {1, 0}});
			break;

		default:
			printf("Bad number of edges %i, valid values are 1, 2, 4, 8 and 16", nedges);
			throw std::exception();
	}
	if (nedges != pattern.size()) {
		printf("error (assert): pattern size incorrect");
		throw std::exception();
	}
	return pattern;
}
