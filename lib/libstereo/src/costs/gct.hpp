#pragma once
#include <opencv2/core.hpp>

#include "../dsbase.hpp"
#include "../array2d.hpp"
#include "census.hpp"

/**
 * Generalized Census Transform
 */

namespace impl {
	template<uint8_t BITS>
	using GeneralizedCensusMatchingCost = NormalizedHammingCost<BITS>;
}

class GeneralizedCensusMatchingCost : public DSBase<impl::GeneralizedCensusMatchingCost<128>> {
public:
	typedef impl::GeneralizedCensusMatchingCost<128> DataType;
	typedef float Type;

	GeneralizedCensusMatchingCost() : DSBase<DataType>(0, 0, 0, 0) {};
	GeneralizedCensusMatchingCost(int width, int height, int disp_min, int disp_max)
		: DSBase<DataType>(width, height, disp_min, disp_max),
			ct_l_(width*data().WSTEP, height), ct_r_(width*data().WSTEP,height)
		{
			data().l = ct_l_.data();
			data().r = ct_r_.data();
		}

	/** Pairs of (x, y) coordinates (relative to window, origin at center)
	 *  indices must fit in signed char [-128,127].
	 *  TODO: Indices must fit within 11x11 window (operator() in
	 * GeneralizedCensusTransform)
	 */
	void setEdges(const std::vector<std::pair<cv::Point2i,cv::Point2i>> &edges);

	void set(cv::InputArray l, cv::InputArray r);
	void set(const Array2D<uchar>& l, const Array2D<uchar>& r);
	static constexpr Type COST_MAX = DataType::COST_MAX;

protected:
	Array2D<uint64_t> ct_l_;
	Array2D<uint64_t> ct_r_;
	Array2D<char> edges_;

	cv::Point2i pmax;
	cv::Point2i pmin;
};

// ==== Pattern generators =====================================================

std::vector<std::pair<cv::Point2i, cv::Point2i>> pattern_dense(const cv::Size size);

std::vector<std::pair<cv::Point2i, cv::Point2i>> pattern_sparse(const cv::Size size, int step=2);

std::vector<std::pair<cv::Point2i, cv::Point2i>> pattern_random(const cv::Size size, int nedges);
std::vector<std::pair<cv::Point2i, cv::Point2i>> pattern_random(const cv::Size size);

/** patterns presented in the original paper */
std::vector<std::pair<cv::Point2i, cv::Point2i>> pattern_gct(int nedges);
