/**
 * @file rollingavg.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#ifndef _FTL_ROLLING_AVERAGE_HPP_
#define _FTL_ROLLING_AVERAGE_HPP_

namespace ftl {
namespace utility {

/**
 * General rolling average class where `SIZE` is the number of items to
 * average over. This is a fast version which may possibily have issues with
 * floating point errors, however these should average out as well. A more
 * accurate version would be much slower.
 * 
 * Unused?
 */
template <typename T, size_t SIZE>
struct RollingAvg {
	RollingAvg() {
		std::fill(vals_, vals_+SIZE, T(0));
	}

	/**
	 * Give a new value to add and return the rolling average including that
	 * new value.
	 */
	float operator()(T v) {
		const size_t mix = (ix_++) % SIZE;
		sum_ = sum_ - vals_[mix] + v;
		vals_[mix] = v;
		return float(sum_) / float(SIZE);
	}

	/** Get current average. */
	inline float value() const { return sum_; }

	private:
	T sum_ = 0;
	T vals_[SIZE] = {0};
	size_t ix_ = 0;
};

}
}

#endif