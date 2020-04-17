#ifndef _FTL_LIBSTEREO_TYPES_HPP_
#define _FTL_LIBSTEREO_TYPES_HPP_

enum AggregationDirections {
	LEFTRIGHT = 1,
	RIGHTLEFT = 2,
	HORIZONTAL = 1+2,
	UPDOWN = 4,
	DOWNUP = 8,
	VERTICAL = 4+8,
	TOPLEFTBOTTOMRIGHT = 16,
	BOTTOMRIGHTTOPLEFT = 32,
	DIAGONAL1 = 16+32,
	BOTTOMLEFTTOPRIGHT = 64,
	TOPRIGHTBOTTOMLEFT = 128,
	DIAGONAL2 = 64+128,
	DIAGONAL = DIAGONAL1+DIAGONAL2,
	ALL = HORIZONTAL+VERTICAL+DIAGONAL,
};

#endif
