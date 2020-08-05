#ifndef _FTL_UTILITY_INTRINSICS_HPP_
#define _FTL_UTILITY_INTRINSICS_HPP_

namespace ftl {

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
