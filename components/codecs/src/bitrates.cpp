
#include <ftl/codecs/bitrates.hpp>
#include <cmath>

using ftl::codecs::CodecPreset;
using ftl::codecs::bitrate_t;
using ftl::codecs::preset_t;
using ftl::codecs::definition_t;
using ftl::codecs::codec_t;


static const CodecPreset special_presets[] = {
	definition_t::HTC_VIVE, bitrate_t::High
};

static const CodecPreset presets[] = {
	definition_t::HD1080, bitrate_t::High,
	definition_t::HD720, bitrate_t::High,
	definition_t::SD576, bitrate_t::High,
	definition_t::SD480, bitrate_t::High,
	definition_t::LD360, bitrate_t::Standard,
	definition_t::LD360, bitrate_t::Low
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

	// TODO error!
	return definition_t::Any;
}

/*
const CodecPreset &ftl::codecs::getPreset(preset_t p) {
	if (p < 0 && p >= -1) return special_presets[std::abs(p+1)];
	if (p > kPresetWorst) return presets[kPresetWorst];
	if (p < kPresetBest) return presets[kPresetBest];
	return presets[p];
}

preset_t ftl::codecs::findPreset(size_t width, size_t height) {
	int min_error = std::numeric_limits<int>::max();

	// Find best definition
	int best_def = (int)definition_t::Invalid;

	for (int i=0; i<(int)definition_t::Invalid; ++i) {
		int dw = resolutions[i].width - width;
		int dh = resolutions[i].height - height;
		int error = dw*dw + dh*dh;
		if (error < min_error) {
			min_error = error;
			best_def = i;
		}
	}

	// Find preset that matches this best definition
	for (preset_t i=kPresetMinimum; i<=kPresetWorst; ++i) {
		const auto &preset = getPreset(i);

		if ((int)preset.res == best_def) {
			return i;
		}
	}

	return kPresetWorst;
}
*/
