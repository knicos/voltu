#include "stereo.hpp"
#include "stereosgm.hpp"
#include "../costs/gct.hpp"

struct StereoGTCensusSgm::Impl : public StereoSgm<GeneralizedCensusMatchingCost, StereoGTCensusSgm::Parameters> {
	Array2D<uchar> l;
	Array2D<uchar> r;

	Impl(StereoGTCensusSgm::Parameters &params, int width, int height, int dmin, int dmax) :
		StereoSgm(params, width, height, dmin, dmax), l(width, height), r(width, height) {}
};

StereoGTCensusSgm::StereoGTCensusSgm() : impl_(nullptr) {
	impl_ = new Impl(params, 0, 0, 0, 0);
}

void StereoGTCensusSgm::compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity) {

	//cudaSetDevice(0);

	if (l.rows() != impl_->cost.height() || r.cols() != impl_->cost.width()) {
		delete impl_; impl_ = nullptr;
		impl_ = new Impl(params, l.cols(), l.rows(), params.d_min, params.d_max);
	}

	mat2gray(l, impl_->l);
	mat2gray(r, impl_->r);

	impl_->cost.setEdges({
		{{-1,-1}, {1,1}},
		{{1,-1}, {-1,1}},
		{{0,-1}, {0,1}},
		{{-1,0}, {1,0}},
		{{-2,-2}, {2,2}},
		{{2,-2}, {-2,2}},
		{{0,-2}, {0,2}},
		{{-2,0}, {2,0}}
	});
	/* wx * wy square window
	std::vector<std::pair<std::pair<int, int>,std::pair<int, int>>> edges;
	{
		int wx = 7;
		int wy = 7;

		for (int x = -wx/2; x <= wx/2; x++) {
			for (int y = -wy/2; y <= wy/2; y++) {
				edges.push_back({{0,0},{y,x}});
			}
		}
	}*/

	impl_->cost.set(impl_->l, impl_->r);

	cudaSafeCall(cudaDeviceSynchronize());
	impl_->compute(disparity);
	/*WinnerTakesAll<GeneralizedCensusMatchingCost> wta;
	wta(impl_->cost, 0, true);

	median_filter(wta.disparity, disparity);*/
}

StereoGTCensusSgm::~StereoGTCensusSgm() {
	if (impl_) {
		delete impl_;
		impl_ = nullptr;
	}
}
