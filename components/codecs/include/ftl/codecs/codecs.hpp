#ifndef _FTL_CODECS_BITRATES_HPP_
#define _FTL_CODECS_BITRATES_HPP_

#include <cstdint>
#include <ftl/utility/msgpack.hpp>

namespace ftl {
namespace codecs {

enum struct format_t {
	BGRA8,
	RGBA8,
	VUYA16,
	F32,
	U16
};

static constexpr uint8_t kFlagFlipRGB = 0x01;		// Swap blue and red channels [deprecated]
static constexpr uint8_t kFlagMappedDepth = 0x02;	// Use Yuv mapping for float [deprecated]
static constexpr uint8_t kFlagFloat = 0x04;			// Floating point output
static constexpr uint8_t kFlagPartial = 0x10;		// This frameset is not complete
static constexpr uint8_t kFlagMultiple = 0x80;		// Multiple video frames in single packet

/**
 * Compression format used.
 */
enum struct codec_t : uint8_t {
	JPG = 0,
	PNG,
	H264,
	HEVC,  // H265
	H264_LOSSLESS,
	HEVC_LOSSLESS,

	// TODO: Add audio codecs
	WAV,

	JSON = 100,		// A JSON string
	CALIBRATION,	// Camera parameters object
	POSE,			// 4x4 eigen matrix
	MSGPACK,
	STRING,			// Null terminated string
	RAW,				// Some unknown binary format

	Invalid = 254,
	Any = 255
};

/**
 * Resolution of encoding.
 */
enum struct definition_t : uint8_t {
	UHD8k = 0,
	UHD4k = 1,
	HD1080 = 2,
	HD720 = 3,
	SD576 = 4,
	SD480 = 5,
	LD360 = 6,
	Any = 7,

	HTC_VIVE = 8,
	OLD_SKOOL = 9,
	MIDDLEBURY = 10,
	MIDDLEBURY_HD = 11,

	hz48000 = 32,
	hz44100 = 33,

	Invalid
};

/**
 * Find exact match definition.
 */
definition_t findDefinition(int width, int height);

/**
 * Find a definition that matches the requested height.
 */
definition_t findDefinition(int height);

/**
 * Get width in pixels of definition.
 */
int getWidth(definition_t);

/**
 * Get height in pixels of definition.
 */
int getHeight(definition_t);

/**
 * General indication of desired quality. Exact bitrate numbers depends also
 * upon chosen definition and codec.
 */
enum struct bitrate_t {
	High,
	Standard,
	Low
};

std::pair<int,int> chooseTileConfig(int size);

}
}

MSGPACK_ADD_ENUM(ftl::codecs::codec_t);
MSGPACK_ADD_ENUM(ftl::codecs::definition_t);

#endif  // _FTL_CODECS_BITRATES_HPP_
