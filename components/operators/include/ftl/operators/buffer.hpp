#ifndef _FTL_OPERATORS_BUFFERS_HPP_
#define _FTL_OPERATORS_BUFFERS_HPP_

namespace ftl {
namespace operators {

/**
 * Similar to frame channels, but these are pipeline buffers that can be
 * used from one operator to the next.
 */
enum class Buffer {
	LowLeft			= 0,	// 8UC4
	Screen			= 1,
	Weights			= 2,	// short
	Confidence		= 3,	// 32F
	Contribution	= 4,	// 32F
	Flow			= 5,	// 16SC2
	Flow2			= 6,	// 16SC2
	Energy			= 7,	// 32F
	Mask			= 8,	// 32U
	Density			= 9,	// 32F
	Support1		= 10,	// 8UC4 (currently)
	Support2		= 11,	// 8UC4 (currently)
	Segmentation	= 12,	// 32S?	
	Disparity		= 13,
	Smoothing		= 14,	// 32F
	LowGrayLeft		= 15,
	LowGrayRight	= 16,
	GrayLeft		= 17,
	GrayRight		= 18,
	LowRight		= 19
};

}
}

#endif