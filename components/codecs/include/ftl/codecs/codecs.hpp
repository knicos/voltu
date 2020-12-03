/**
 * @file codecs.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#ifndef _FTL_CODECS_CODECS_HPP_
#define _FTL_CODECS_CODECS_HPP_

#include <cstdint>
#include <msgpack.hpp>

namespace ftl {

/**
 * Video and data encoding / decoding components are located in this namespace. 
 * Audio codecs are for now in `ftl::audio` namespace.
 */
namespace codecs {

// TODO: (nick) Some of these have or should be moved to encoder/decoder implementation

static constexpr uint8_t kFlagFlipRGB = 0x01;		///< Swap blue and red channels [deprecated]
static constexpr uint8_t kFlagMappedDepth = 0x02;	///< Use Yuv mapping for float [deprecated]
static constexpr uint8_t kFlagFloat = 0x04;			///< Floating point output
static constexpr uint8_t kFlagPartial = 0x10;		///< This frameset is not complete
static constexpr uint8_t kFlagStereo = 0x20;		///< Left-Right stereo in single channel
static constexpr uint8_t kFlagMultiple = 0x80;		///< Multiple video frames in single packet

static constexpr uint8_t kFlagRequest = 0x01;		///< Used for empty data packets to mark a request for data
static constexpr uint8_t kFlagCompleted = 0x02;		///< Last packet for timestamp
static constexpr uint8_t kFlagReset = 0x04;

/**
 * Compression format used.
 */
enum struct codec_t : uint8_t {  // TODO: Rename to Codec?
	/* Video (image) codecs */
	JPG = 0,
	PNG,
	H264,
	HEVC,  			// H265
	H264_LOSSLESS,
	HEVC_LOSSLESS,

	/* Audio codecs */
	WAV=32,
	OPUS,

	/* Data "codecs" */
	JSON = 100,		// A JSON string
	CALIBRATION,	// Camera parameters object
	POSE,			// 4x4 eigen matrix
	MSGPACK,
	STRING,			// Null terminated string
	RAW,			// Some unknown binary format

	Invalid = 254,
	Any = 255
};

/** Given a frame count, return a width x height tile configuration. */
std::pair<int,int> chooseTileConfig(int size);

}  // namespace codecs
}  // namespace ftl

MSGPACK_ADD_ENUM(ftl::codecs::codec_t);

#endif  // _FTL_CODECS_CODECS_HPP_
