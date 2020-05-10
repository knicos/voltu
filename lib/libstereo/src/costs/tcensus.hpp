#ifndef _FTL_LIBSTEREO_COSTS_TCENSUS_HPP_
#define _FTL_LIBSTEREO_COSTS_TCENSUS_HPP_

#include <opencv2/core/core.hpp>
#include "array2d.hpp"
#include "dsbase.hpp"
#include "census.hpp"

#include <cuda_runtime.h>

class TCensusMatchingCost : public DSBase<impl::CensusMatchingCost<9,7,2>> {
public:
	typedef impl::CensusMatchingCost<9,7,2> DataType;
	typedef unsigned short Type;

	TCensusMatchingCost() : DSBase<DataType>(0, 0, 0, 0) {};
	TCensusMatchingCost(int width, int height, int disp_min, int disp_max)
		: DSBase<DataType>(width, height, disp_min, disp_max),
			ct_l_(width*data().WSTEP, height), ct_r_(width*data().WSTEP,height), t_(0)
		{
			data().l = ct_l_.data();
			data().r = ct_r_.data();
		}

	void set(cv::InputArray l, cv::InputArray r);
	void set(const Array2D<uchar>& l, const Array2D<uchar>& r);
	void setT(uchar t) { t_ = t; }

	static constexpr Type  COST_MAX = DataType::COST_MAX;

protected:
	Array2D<uint64_t> ct_l_;
	Array2D<uint64_t> ct_r_;
	uchar t_;
};

#endif
