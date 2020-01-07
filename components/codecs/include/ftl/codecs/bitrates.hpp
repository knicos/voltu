#ifndef _FTL_CODECS_BITRATES_HPP_
#define _FTL_CODECS_BITRATES_HPP_

#include <cstdint>
#include <msgpack.hpp>

namespace ftl {
namespace codecs {

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

	// TODO: Add audio definitions

	Invalid
};

definition_t findDefinition(int width, int height);

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

/**
 * Pre-specified definition and quality levels for encoding where kPreset0 is
 * the best quality and kPreset9 is the worst. The use of presets is useful for
 * adaptive bitrate scenarios where the numbers are increased or decreased.
 */
typedef int8_t preset_t;
static const preset_t kPreset0 = 0;
static const preset_t kPreset1 = 1;
static const preset_t kPreset2 = 2;
static const preset_t kPreset3 = 3;
static const preset_t kPreset4 = 4;
static const preset_t kPreset5 = 5;
static const preset_t kPreset6 = 6;
static const preset_t kPreset7 = 7;
static const preset_t kPreset8 = 8;
static const preset_t kPreset9 = 9;
static const preset_t kPresetBest = 0;
static const preset_t kPresetWorst = 9;
static const preset_t kPresetLQThreshold = 4;

static const preset_t kPresetHTCVive = -1;

static const preset_t kPresetMinimum = -1;

/**
 * Represents the details of each preset codec configuration.
 */
struct CodecPreset {
	definition_t res;
	bitrate_t qual;
};

/**
 * Get preset details structure from preset number.
 */
const CodecPreset &getPreset(preset_t);

/**
 * Get preset based upon a requested definition. If multiple presets match then
 * the highest quality one is returned.
 */
const CodecPreset &getPreset(definition_t, definition_t);

/**
 * Get preset based upon a requested definition for colour channel.
 * If multiple presets match then the highest quality one is returned.
 */
const CodecPreset &getPreset(definition_t);

/**
 * Get the preset id nearest to requested definition for colour and depth.
 */
preset_t findPreset(definition_t, definition_t);

/**
 * Get the preset id nearest to requested colour definition. If multiple presets
 * match then return highest quality.
 */
preset_t findPreset(definition_t);

preset_t findPreset(size_t width, size_t height);

}
}

MSGPACK_ADD_ENUM(ftl::codecs::codec_t);
MSGPACK_ADD_ENUM(ftl::codecs::definition_t);

#endif  // _FTL_CODECS_BITRATES_HPP_
