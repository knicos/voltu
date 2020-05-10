#ifndef _FTL_LIBSTEREO_COSTS_STABLE_HPP_
#define _FTL_LIBSTEREO_COSTS_STABLE_HPP_

#include "census.hpp"

/**
 * STABLE descriptor matching cost
 */
class StableMatchingCost : public DSBase<impl::HammingCost<16>> {
public:
	typedef impl::HammingCost<16> DataType;
	typedef DataType::Type Type;

	StableMatchingCost() : DSBase<DataType>(0, 0, 0, 0) {};
	StableMatchingCost(int width, int height, int disp_min, int disp_max)
		: DSBase<DataType>(width, height, disp_min, disp_max),
			wsize_(0), filter_mask_(0, 0),
			stable_l_(width*data().WSTEP, height),
			stable_r_(width*data().WSTEP, height)
		{
			data().l = stable_l_.data();
			data().r = stable_r_.data();
		}

	void generateFilterMask(const int wsize, const int bits);
	void set(cv::InputArray l, cv::InputArray r);
	void set(const Array2D<uchar>& l, const Array2D<uchar>& r);
	void set(const Array2D<uchar>& l, const Array2D<uchar>& r, size_t w, size_t h);
	static constexpr Type COST_MAX = DataType::COST_MAX;

	Array2D<int16_t> &getFilter() { return filter_mask_; }
	void setFilter(const Array2D<int16_t> &f) { filter_mask_ = f; }

protected:
	int wsize_;
	Array2D<int16_t> filter_mask_;
	Array2D<uint64_t> stable_l_;
	Array2D<uint64_t> stable_r_;
};

#endif
