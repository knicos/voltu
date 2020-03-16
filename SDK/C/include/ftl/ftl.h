#ifndef _FTL_SDK_HPP_
#define _FTL_SDK_HPP_

#ifdef __cplusplus
#include <cstdint>
#else
#include <stdint.h>
#endif

struct FTLStream;
typedef FTLStream* ftlStream_t;


enum ftlError_t {
	FTLERROR_OK,
	FTLERROR_UNKNOWN,
	FTLERROR_STREAM_READONLY,
	FTLERROR_STREAM_WRITEONLY,
	FTLERROR_STREAM_NO_FILE,
	FTLERROR_STREAM_LISTEN_FAILED,
	FTLERROR_STREAM_FILE_CREATE_FAILED,
	FTLERROR_STREAM_NET_CREATE_FAILED,
	FTLERROR_STREAM_INVALID_STREAM,
	FTLERROR_STREAM_INVALID_PARAMETER,
	FTLERROR_STREAM_ENCODE_FAILED,
	FTLERROR_STREAM_DECODE_FAILED,
	FTLERROR_STREAM_BAD_CHANNEL,
	FTLERROR_STREAM_BAD_TIMESTAMP,
	FTLERROR_STREAM_BAD_URI,
	FTLERROR_STREAM_BAD_IMAGE_TYPE,
	FTLERROR_STREAM_BAD_DATA,
	FTLERROR_STREAM_BAD_IMAGE_SIZE,
	FTLERROR_STREAM_NO_INTRINSICS,
	FTLERROR_STREAM_NO_DATA,
	FTLERROR_STREAM_DUPLICATE
};

enum ftlChannel_t {
	FTLCHANNEL_None				= -1,
	FTLCHANNEL_Colour			= 0,	// 8UC3 or 8UC4
	FTLCHANNEL_Left				= 0,
	FTLCHANNEL_Depth			= 1,	// 32S or 32F
	FTLCHANNEL_Right			= 2,	// 8UC3 or 8UC4
	FTLCHANNEL_Colour2			= 2,
	FTLCHANNEL_Depth2			= 3,
	FTLCHANNEL_Deviation		= 4,
	FTLCHANNEL_Screen			= 4,
	FTLCHANNEL_Normals			= 5,	// 16FC4
	FTLCHANNEL_Weights			= 6,	// short
	FTLCHANNEL_Confidence		= 7,	// 32F
	FTLCHANNEL_Contribution		= 7,	// 32F
	FTLCHANNEL_EnergyVector		= 8,	// 32FC4
	FTLCHANNEL_Flow				= 9,	// 32F
	FTLCHANNEL_Energy			= 10,	// 32F
	FTLCHANNEL_Mask				= 11,	// 32U
	FTLCHANNEL_Density			= 12,	// 32F
	FTLCHANNEL_Support1			= 13,	// 8UC4 (currently)
	FTLCHANNEL_Support2			= 14,	// 8UC4 (currently)
	FTLCHANNEL_Segmentation		= 15,	// 32S?
	FTLCHANNEL_Normals2			= 16,	// 16FC4
	FTLCHANNEL_ColourHighRes	= 17,	// 8UC3 or 8UC4
	FTLCHANNEL_LeftHighRes		= 17,	// 8UC3 or 8UC4
	FTLCHANNEL_Disparity		= 18,
	FTLCHANNEL_Smoothing		= 19,	// 32F
	FTLCHANNEL_RightHighRes		= 20,	// 8UC3 or 8UC4
	FTLCHANNEL_Colour2HighRes	= 20,
	FTLCHANNEL_Overlay			= 21,   // 8UC4
	FTLCHANNEL_GroundTruth		= 22,	// 32F

	FTLCHANNEL_Audio			= 32,
	FTLCHANNEL_AudioMono		= 32,
	FTLCHANNEL_AudioStereo		= 33,

	FTLCHANNEL_Configuration	= 64,	// JSON Data
	FTLCHANNEL_Settings1		= 65,
	FTLCHANNEL_Calibration		= 65,	// Camera Parameters Object
	FTLCHANNEL_Pose				= 66,	// Eigen::Matrix4d
	FTLCHANNEL_Settings2		= 67,
	FTLCHANNEL_Calibration2		= 67,	// Right camera parameters
	FTLCHANNEL_Index           	= 68,
	FTLCHANNEL_Control			= 69,	// For stream and encoder control
	FTLCHANNEL_Settings3		= 70,

	FTLCHANNEL_Data				= 2048,	// Custom data, any codec.
	FTLCHANNEL_Faces			= 2049, // Data about detected faces
	FTLCHANNEL_Transforms		= 2050,	// Transformation matrices for framesets
	FTLCHANNEL_Shapes3D			= 2051,	// Labeled 3D shapes
	FTLCHANNEL_Messages			= 2052	// Vector of Strings
};

enum ftlImageFormat_t {
	FTLIMAGE_FLOAT,
	FTLIMAGE_BGRA,
	FTLIMAGE_RGBA,
	FTLIMAGE_RGB,
	FTLIMAGE_BGR,
	FTLIMAGE_RGB_FLOAT		// Used by Blender
};

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Get the last error for a particular stream. Or use NULL stream to get errors
 * when creating streams.
 */
ftlError_t ftlGetLastStreamError(ftlStream_t stream);

// ==== FTL Stream API =========================================================

/**
 * Create a new file or net stream from a URI. This is for writing or sending
 * data and makes a write only stream.
 */
ftlStream_t ftlCreateWriteStream(const char *uri);

/**
 * Open an existing file or network stream from the URI. These streams are
 * read only.
 */
ftlStream_t ftlCreateReadStream(const char *uri);

/**
 * Write raw image data to a frame. The width and height of
 * the image contained in `data` are contained within the intrinsics data which
 * must be written to a stream before calling `ftlImageWrite`. The `pitch`
 * argument determines the number of bytes per row of the image, if set to 0
 * the pitch is assumed to be width multiplied by size of pixel in bytes. Note
 * that `ftlNextFrame` must be called before calling this again for the same
 * source and channel.
 * 
 * @param stream As created with `ftlCreateWriteStream`
 * @param sourceId Unique consecutive ID for a camera or source
 * @param channel The image channel
 * @param type The image format
 * @param pitch Bytes per row, in case different from width x sizeof(type)
 * @param data Raw image data, pitch x height bytes in size
 * 
 * @return FTLERROR_OK on success
 */
ftlError_t ftlImageWrite(
	ftlStream_t stream,
	int32_t sourceId,
	ftlChannel_t channel,
	ftlImageFormat_t type,
	uint32_t pitch,
	const void *data);

/**
 * Only for writing streams, this determines the timestamp interval between
 * frames when `ftlNextFrame` is called. When reading an FTL file this is
 * determined automatically. Default is 25 fps. Advised to call this only once
 * before any call to `ftlNextFrame`.
 * 
 * @param stream As created with `ftlCreateWriteStream`
 * @param fps Frames per second of video output
 */
ftlError_t ftlSetFrameRate(ftlStream_t stream, float fps);

/**
 * Move a write stream to the next frame. It is an error to call
 * `ftlImageWrite` multiple times for the same source and channel without
 * calling this function. This will also be used when reading FTL files.
 */
ftlError_t ftlNextFrame(ftlStream_t stream);

/**
 * Write of 4x4 transformation matrix into the stream for a given source. The
 * `data` pointer must contain 16 float values packed and in Eigen::Matrix4f
 * form.
 */
ftlError_t ftlPoseWrite(ftlStream_t stream, int32_t sourceId, const float *data);

/**
 * This should be called before any other function for a given `sourceId`.
 * The width and height information here is used implicitely by other API calls.
 */
ftlError_t ftlIntrinsicsWriteLeft(ftlStream_t stream, int32_t sourceId, int32_t width, int32_t height, float f, float cx, float cy, float baseline, float minDepth, float maxDepth);

/**
 * Call this after the left intrinsics have been set.
 */
ftlError_t ftlIntrinsicsWriteRight(ftlStream_t stream, int32_t sourceId, int32_t width, int32_t height, float f, float cx, float cy, float baseline, float minDepth, float maxDepth);

/**
 * Close and destroy the stream, ensuring all read/write operations have
 * completed. Network connections are terminated and files closed.
 */
ftlError_t ftlDestroyStream(ftlStream_t stream);

#ifdef __cplusplus
};
#endif

#endif
