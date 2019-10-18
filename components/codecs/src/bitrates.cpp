
#include <ftl/codecs/bitrates.hpp>
#include <cmath>

using ftl::codecs::CodecPreset;
using ftl::codecs::bitrate_t;
using ftl::codecs::preset_t;
using ftl::codecs::definition_t;
using ftl::codecs::codec_t;

static const CodecPreset special_presets[] = {
	definition_t::HTC_VIVE, definition_t::HTC_VIVE, bitrate_t::High, bitrate_t::High
};

static const CodecPreset presets[] = {
	definition_t::HD1080, definition_t::HD1080, bitrate_t::High, bitrate_t::High,
	definition_t::HD1080, definition_t::HD720, bitrate_t::Standard, bitrate_t::Standard,
	definition_t::HD720, definition_t::HD720, bitrate_t::High, bitrate_t::High,
	definition_t::HD720, definition_t::SD576, bitrate_t::Standard, bitrate_t::Standard,
	definition_t::SD576, definition_t::SD576, bitrate_t::High, bitrate_t::High,
	definition_t::SD576, definition_t::SD480, bitrate_t::Standard, bitrate_t::Standard,
	definition_t::SD480, definition_t::SD480, bitrate_t::High, bitrate_t::High,
	definition_t::SD480, definition_t::LD360, bitrate_t::Standard, bitrate_t::Standard,
	definition_t::LD360, definition_t::LD360, bitrate_t::Standard, bitrate_t::Standard,
	definition_t::LD360, definition_t::LD360, bitrate_t::Low, bitrate_t::Low
};

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
	1852, 2056		// HTC_VIVE
};

int ftl::codecs::getWidth(definition_t d) {
	return resolutions[static_cast<int>(d)].width;
}

int ftl::codecs::getHeight(definition_t d) {
	return resolutions[static_cast<int>(d)].height;
}

const CodecPreset &ftl::codecs::getPreset(preset_t p) {
	if (p < 0 && p >= -1) return special_presets[std::abs(p+1)];
    if (p > kPresetWorst) return presets[kPresetWorst];
    if (p < kPresetBest) return presets[kPresetBest];
    return presets[p];
};
