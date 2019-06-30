#ifndef _FTL_RGBD_BITRATESETTINGS_HPP_
#define _FTL_RGBD_BITRATESETTINGS_HPP_

namespace ftl {
namespace rgbd {
namespace detail {

struct BitrateSetting {
	int width;
	int height;
	int jpg_quality;
	int png_compression;
};

static const BitrateSetting bitrate_settings[] = {
	1280, 720, 95, 1,
	1280, 720, 95, 1,
	1280, 720, 95, 1,
	1280, 720, 75, 1,
	640, 360, 95, 1,
	640, 360, 75, 5,
	640, 360, 50, 5,
	320, 160, 95, 5,
	320, 160, 75, 5,
	320, 160, 50, 9
};

}
}
}

#endif  // _FTL_RGBD_BITRATESETTINGS_HPP_
