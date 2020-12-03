/**
 * @file intrinsics.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#ifndef _FTL_UTILITY_INTRINSICS_HPP_
#define _FTL_UTILITY_INTRINSICS_HPP_

namespace ftl {

/** Count number of set bits in 64bit unsigned int. */
inline unsigned int popcount(uint64_t bits) {
	#if defined(_MSC_VER)
		return __popcnt64(bits);
	#elif defined(__GNUC__)
		return __builtin_popcountl(bits);
	#else
		int count = 0;
		while (bits != 0) {
			bits = bits >> 1;
			count += uint64_t(1) & bits;
		}
		return count;
	#endif
}

/** Count number o set bits in 32bit unsigned int */
inline unsigned int popcount(uint32_t bits) {
	#if defined(_MSC_VER)
		return __popcnt(bits);
	#elif defined(__GNUC__)
		return __builtin_popcount(bits);
	#else
		int count = 0;
		while (bits != 0) {
			bits = bits >> 1;
			count += uint32_t(1) & bits;
		}
		return count;
	#endif
}

}

#endif
