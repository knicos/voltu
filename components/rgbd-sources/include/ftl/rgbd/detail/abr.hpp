#ifndef _FTL_RGBD_ABR_HPP_
#define _FTL_RGBD_ABR_HPP_

#include <ftl/rgbd/detail/netframe.hpp>
#include <cstdint>

namespace ftl {
namespace rgbd {
namespace detail {

static const float kAspectRatio = 1.777778f;

enum codec_t {
	kCodecJPG = 0,
	kCodecPNG
};

struct BitrateSetting {
	int colour_res;
	int depth_res;
	int colour_qual;
	int depth_qual;
	codec_t colour_codec;
	codec_t depth_codec;
	int block_count_x;

	/*int width;
	int height;
	int jpg_quality;
	int png_compression;
	codec_t colour_codec;
	codec_t depth_codec;
	int chunking;*/
};

static const BitrateSetting bitrate_settings[] = {
	1080, 1080, 95, 1, kCodecJPG, kCodecPNG, 4,
	1080, 720, 95, 1, kCodecJPG, kCodecPNG, 4,
	720, 720, 95, 1, kCodecJPG, kCodecPNG, 4,
	720, 576, 95, 5, kCodecJPG, kCodecPNG, 4,
	576, 576, 95, 5, kCodecJPG, kCodecPNG, 4,
	576, 480, 95, 5, kCodecJPG, kCodecPNG, 2,
	480, 480, 95, 5, kCodecJPG, kCodecPNG, 2,
	480, 360, 95, 9, kCodecJPG, kCodecPNG, 2,
	360, 360, 95, 9, kCodecJPG, kCodecPNG, 2,
	360, 360, 50, 9, kCodecJPG, kCodecPNG, 2
};

/*static const BitrateSetting bitrate_settings[] = {
	1920, 1080, 95, 1, kCodecJPG, kCodecPNG, 4,	// ?
	1280, 720, 95, 1, kCodecJPG, kCodecPNG, 4,	// ~96Mbps
	1024, 576, 95, 5, kCodecJPG, kCodecPNG, 3,	// ~62Mbps
	854, 480, 95, 5, kCodecJPG, kCodecPNG, 3,	// ~48Mbps
	640, 360, 95, 9, kCodecJPG, kCodecPNG, 2,	// ~31Mbps
	640, 360, 75, 9, kCodecJPG, kCodecPNG, 2,	// ~25Mbps
	640, 360, 65, 9, kCodecJPG, kCodecPNG, 2,	// ~24Mbps
	640, 360, 50, 9, kCodecJPG, kCodecPNG, 2,	// ~23Mbps
	320, 160, 95, 9, kCodecJPG, kCodecPNG, 2,	// ~10Mbps
	320, 160, 75, 9, kCodecJPG, kCodecPNG, 2	// ~8Mbps
};*/

typedef unsigned int bitrate_t;

static const bitrate_t kBitrateBest = 0;
static const bitrate_t kBitrateWorst = 9;

/**
 * Adaptive Bitrate Controller to monitor and decide on a client streams
 * bitrate. The basics of our approach are that if transmission latency exceeds
 * some proportion of the frame time then mark it as a slow frame. Similarly if
 * transmission latency falls below a proportion of frame time then mark it as
 * a fast frame. If the net frame status is slow (thresholded) then reduce
 * bitrate, if the net status is fast then increase bitrate.
 */
class ABRController {
	public:
	ABRController();
	~ABRController();

	/**
	 * From a received frame, select a bitrate based upon actual and required
	 * bitrate as well as past frames.
	 */
	bitrate_t selectBitrate(const ftl::rgbd::detail::NetFrame &);

	/**
	 * Called to tell the controller the new bitrate is now in use by the stream
	 */
	void notifyChanged();

	void setMaximumBitrate(bitrate_t);
	void setMinimumBitrate(bitrate_t);

	static const ftl::rgbd::detail::BitrateSetting &getBitrateInfo(bitrate_t b);
	static int getColourWidth(bitrate_t b);
	static int getDepthWidth(bitrate_t b);
	static int getColourHeight(bitrate_t b);
	static int getDepthHeight(bitrate_t b);
	static int getBlockCountX(bitrate_t b);
	static int getBlockCountY(bitrate_t b);
	static int getBlockCount(bitrate_t b);
	static int getColourQuality(bitrate_t b);
	static int getDepthQuality(bitrate_t b);

	private:
	unsigned int down_log_;		// Bit log of delayed frames
	unsigned int up_log_;		// Bit log of fast frames
	int64_t last_br_change_;	// Time of last adaptive change
	float down_threshold_;		// Proportion of min bitrate before reduction
	float up_threshold_;		// Proportion of min bitrate before increase
	bitrate_t bitrate_;
	bool enabled_;
	bitrate_t max_;
	bitrate_t min_;
};

}
}
}

#endif  // _FTL_RGBD_ABR_HPP_
