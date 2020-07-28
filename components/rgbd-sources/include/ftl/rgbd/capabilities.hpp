#ifndef _FTL_RGBD_CAPABILITIES_HPP_
#define _FTL_RGBD_CAPABILITIES_HPP_

#include <ftl/utility/msgpack.hpp>

namespace ftl {
namespace rgbd {

/**
 * To be added to the capabilities channel to indicate what the source device
 * is capable of. These properties should be features of the source that
 * cannot be determined by simply checking for channels, and may include
 * status information about processing that has been performed.
 */
enum class Capability : int {
	MOVABLE=0,	// Is a pose controllable camera
	ACTIVE,		// An active depth sensor
	VIDEO,		// Is video and not just static
	ADJUSTABLE,	// Camera properties can be changed (exposure etc)
	VIRTUAL,	// Is not a physical camera
	TOUCH,		// Touch related feedback supported
	VR,			// Is a VR device, so provides own active pose etc
	LIVE,		// Live, not recorded (removed from ftl file sources)
	FUSED,		// Reconstruction has been performed
	STREAMED,	// Means it came from a stream and not device
	EQUI_RECT,	// 360 rendered (Equirectangular Render)
	STEREO		// Side-by-side stereo render
};

std::string capabilityName(Capability);

}
}

MSGPACK_ADD_ENUM(ftl::rgbd::Capability);

#endif