#pragma once

#include "../dsbase.hpp"
#include "../array2d.hpp"
#include "census.hpp"

namespace impl {
	template<uint8_t WMAX>
	using GeneralizedCensusMatchingCost = HammingCost<WMAX*WMAX>;
}

class GeneralizedCensusMatchingCost : public DSBase<impl::GeneralizedCensusMatchingCost<11>> {
public:
	typedef impl::GeneralizedCensusMatchingCost<11> DataType;
	typedef unsigned short Type;

	GeneralizedCensusMatchingCost() : DSBase<DataType>(0, 0, 0, 0) {};
	GeneralizedCensusMatchingCost(int width, int height, int disp_min, int disp_max)
		: DSBase<DataType>(width, height, disp_min, disp_max),
			ct_l_(width*data().WSTEP, height), ct_r_(width*data().WSTEP,height)
		{
			data().l = ct_l_.data();
			data().r = ct_r_.data();
		}

	// pairs of indices (window size 11x11, origin top left)
	void setEdges(const std::vector<std::pair<int, int>> &edges);
	// pairs of (y,x) coordinates (relative to window, origin at center)
	void setEdges(const std::vector<std::pair<std::pair<int, int>,std::pair<int, int>>> &edges);

	void set(cv::InputArray l, cv::InputArray r);
	void set(const Array2D<uchar>& l, const Array2D<uchar>& r);
	static constexpr Type COST_MAX = DataType::COST_MAX;

protected:
	Array2D<uint64_t> ct_l_;
	Array2D<uint64_t> ct_r_;
	Array2D<uchar> edges_;
};
