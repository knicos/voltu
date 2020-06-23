#include "gct.hpp"
#include "../util.hpp"

static const int WX = 11;
static const int WY = 11;

namespace algorithms {
	/** Fife, W. S., & Archibald, J. K. (2012). Improved census transforms for
	* resource-optimized stereo vision. IEEE Transactions on Circuits and
	* Systems for Video Technology, 23(1), 60-73.
	*/
	template<int WINX, int WINY>
	struct GeneralizedCensusTransform {
		__host__ __device__ inline void window(const int y, const int x, uint64_t* __restrict__ out) {

			uint8_t i = 0; // bit counter for *out

			for (int e = 0; e < nedges; e++) {

				// edges contain window indices, calculate window coordinates
				const int y1 = y + edges(e,0) % WINY - WINY/2;
				const int x1 = x + edges(e,0) / WINY - WINX/2;

				const int y2 = y + edges(e,1) % WINY - WINY/2;
				const int x2 = x + edges(e,1) / WINY - WINX/2;

				// zero if first value, otherwise shift to left
				if (i % 64 == 0) { *out = 0; }
				else             { *out = (*out << 1); }
				*out |= (im(y1,x1) < (im(y2,x2)) ? 1 : 0);

				i += 1;
				// if all bits set, continue to next element
				if (i % 64 == 0) { out++; }
			}
		}

		__host__ __device__  void operator()(ushort2 thread, ushort2 stride, ushort2 size) {
			for (int y = thread.y+WINY/2; y<size.y-WINY/2-1; y+=stride.y) {
				for (int x = thread.x+WINX/2; x<size.x-WINX/2-1; x+=stride.x) {
					window(y, x, &(out(y, x*WSTEP)));
				}
			}
		}

		int nedges;
		Array2D<uchar>::Data edges;
		Array2D<uchar>::Data im;
		Array2D<uint64_t>::Data out;

		// number of uint64_t values for each window
		static constexpr int WSTEP = (WINX*WINY - 1)/(sizeof(uint64_t)*8) + 1;
	};
}

void GeneralizedCensusMatchingCost::set(const Array2D<uchar> &l, const Array2D<uchar> &r) {

	parallel2D<algorithms::GeneralizedCensusTransform<WX,WY>>({edges_.height, edges_.data(), l.data(), ct_l_.data()}, l.width, l.height);
	parallel2D<algorithms::GeneralizedCensusTransform<WX,WY>>({edges_.height, edges_.data(), r.data(), ct_r_.data()}, r.width, r.height);
}

void GeneralizedCensusMatchingCost::setEdges(const std::vector<std::pair<int, int>> &edges) {
	if (edges.size() >= (WX * WY)) {
		throw std::exception(); // too many edges
	}

	cv::Mat data(cv::Size(2, edges.size()), CV_8UC1);
	for (size_t i = 0; i < edges.size(); i++) {
		if (edges[i].first < 0 || edges[0].second < 0) {
			throw std::exception(); // indices must be positive
		}

		data.at<uchar>(i,0) = edges[i].first;
		data.at<uchar>(i,1) = edges[i].second;
	}

	edges_.create(2, edges.size());
	#ifdef USE_GPU
	edges_.toGpuMat().upload(data);
	#else
	data.copyTo(edges_.toMat());
	#endif
}

void GeneralizedCensusMatchingCost::setEdges(const std::vector<std::pair<std::pair<int, int>,std::pair<int, int>>> &edges) {
	std::vector<std::pair<int, int>> edges_idx;
	for (const auto& p : edges) {
		const auto& p1 = p.first;
		const auto& p2 = p.second;

		int i1 = p1.first * WX + p1.second + (WX*WY-1)/2;
		int i2 = p2.first * WX + p2.second + (WX*WY-1)/2;
		printf("i1: %i, i2: %i\n", i1, i2);
		edges_idx.push_back({i1, i2});
	}

	/* TODO: move to unit test
	for (int i = 0; i < edges.size(); i++) {
		auto p = edges_idx[i];

		const int y1 = p.first % WY - WY/2;
		const int x1 = p.first / WY - WX/2;

		const int y2 = p.second % WY - WY/2;
		const int x2 = p.second / WY - WX/2;
		printf("(%i,%i), (%i,%i)\n", y1, x1, y2, x2);
	}*/

	setEdges(edges_idx);
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
		throw std::exception();
	}
}

////////////////////////////////////////////////////////////////////////////////
/*
struct StereoCensusSgm::Impl : public StereoSgm<CensusMatchingCost, StereoCensusSgm::Parameters> {
	Array2D<uchar> l;
	Array2D<uchar> r;

	Impl(StereoCensusSgm::Parameters &params, int width, int height, int dmin, int dmax) :
		StereoSgm(params, width, height, dmin, dmax), l(width, height), r(width, height) {}
};

StereoCensusSgm::StereoCensusSgm() : impl_(nullptr) {
	impl_ = new Impl(params, 0, 0, 0, 0);
}

void StereoCensusSgm::compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity) {

	//cudaSetDevice(0);

	if (l.rows() != impl_->cost.height() || r.cols() != impl_->cost.width()) {
		delete impl_; impl_ = nullptr;
		impl_ = new Impl(params, l.cols(), l.rows(), params.d_min, params.d_max);
	}

	mat2gray(l, impl_->l);
	mat2gray(r, impl_->r);
	impl_->cost.setPattern(params.pattern);
	impl_->cost.set(impl_->l, impl_->r);

	cudaSafeCall(cudaDeviceSynchronize());
	impl_->compute(disparity);
}

StereoCensusSgm::~StereoCensusSgm() {
	if (impl_) {
		delete impl_;
		impl_ = nullptr;
	}
}
*/
