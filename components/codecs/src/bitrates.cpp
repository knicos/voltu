
#include <ftl/codecs/codecs.hpp>
#include <cmath>

using ftl::codecs::definition_t;
using ftl::codecs::codec_t;


static const float kAspectRatio = 1.777778f;

struct Resolution {
	int width;
	int height;
};

static const Resolution resolutions[] = {
	7680, 4320,		// UHD8k
	3840, 2160,		// UHD4k
	1920, 1080,		// HD1080
	1280, 720,		// HD720
	1024, 576,		// SD576
	854, 480,		// SD480
	640, 360,		// LD360
	0, 0,			// ANY
	1852, 2056,		// HTC_VIVE
	0, 0
};

int ftl::codecs::getWidth(definition_t d) {
	return resolutions[static_cast<int>(d)].width;
}

int ftl::codecs::getHeight(definition_t d) {
	return resolutions[static_cast<int>(d)].height;
}

definition_t ftl::codecs::findDefinition(int width, int height) {
	int best = 0;

	for(const Resolution res : resolutions) {
		if ((res.width == width) && (res.height == height)) {
			return static_cast<definition_t>(best);
		}
		best++;
	}

	return definition_t::Invalid;
}

