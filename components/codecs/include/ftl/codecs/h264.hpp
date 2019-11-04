#ifndef _FTL_CODECS_H264_HPP_
#define _FTL_CODECS_H264_HPP_

namespace ftl {
namespace codecs {

/**
 * H.264 codec utility functions.
 */
namespace h264 {

/**
 * H264 Network Abstraction Layer Unit types.
 */
enum class NALType : int {
	UNSPECIFIED_0 = 0,
	CODED_SLICE_NON_IDR = 1,
	CODED_SLICE_PART_A = 2,
	CODED_SLICE_PART_B = 3,
	CODED_SLICE_PART_C = 4,
	CODED_SLICE_IDR = 5,
	SEI = 6,
	SPS = 7,
	PPS = 8,
	ACCESS_DELIMITER = 9,
	EO_SEQ = 10,
	EO_STREAM = 11,
	FILTER_DATA = 12,
	SPS_EXT = 13,
	PREFIX_NAL_UNIT = 14,
	SUBSET_SPS = 15,
	RESERVED_16 = 16,
	RESERVED_17 = 17,
	RESERVED_18 = 18,
	CODED_SLICE_AUX = 19,
	CODED_SLICE_EXT = 20,
	CODED_SLICE_DEPTH = 21,
	RESERVED_22 = 22,
	RESERVED_23 = 23,
	UNSPECIFIED_24 = 24,
	UNSPECIFIED_25,
	UNSPECIFIED_26,
	UNSPECIFIED_27,
	UNSPECIFIED_28,
	UNSPECIFIED_29,
	UNSPECIFIED_30,
	UNSPECIFIED_31
};

/**
 * Extract the NAL unit type from the first NAL header.
 * With NvPipe, the 5th byte contains the NAL Unit header.
 */
inline NALType getNALType(const std::vector<uint8_t> &data) {
	return static_cast<NALType>(data[4] & 0x1F);
}

/**
 * Check the H264 bitstream for an I-Frame. With NvPipe, all I-Frames start
 * with a SPS NAL unit so just check for this.
 */
inline bool isIFrame(const std::vector<uint8_t> &data) {
	return getNALType(data) == NALType::SPS;
}

}
}
}

#endif  // _FTL_CODECS_H264_HPP_
