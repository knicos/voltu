#include <ftl/codecs/encoder.hpp>
#include <ftl/threads.hpp>

#include <list>

#include <ftl/config.h>
#include <loguru.hpp>

using ftl::codecs::Encoder;
using ftl::codecs::bitrate_t;
using ftl::codecs::definition_t;
using ftl::codecs::preset_t;
using ftl::codecs::device_t;
using ftl::codecs::codec_t;
using ftl::codecs::kPresetBest;
using ftl::codecs::kPresetWorst;
using ftl::codecs::kPresetLQThreshold;

namespace ftl {
namespace codecs {
namespace internal {

std::list<Encoder*> encoders;

bool has_been_init = false;

void init_encoders();

}
}
}

using namespace ftl::codecs::internal;

static MUTEX mutex;

Encoder *ftl::codecs::allocateEncoder(ftl::codecs::definition_t maxdef,
		ftl::codecs::device_t dev, ftl::codecs::codec_t codec) {
    UNIQUE_LOCK(mutex, lk);
	if (!has_been_init) init_encoders();

	for (auto i=encoders.begin(); i!=encoders.end(); ++i) {
		auto *e = *i;
		if (!e->available) continue;
		if (dev != device_t::Any && dev != e->device) continue;
		if (maxdef != definition_t::Any && (maxdef < e->max_definition || maxdef > e->min_definition)) continue;
		if (codec != codec_t::Any && !e->supports(codec)) continue;
		
		e->available = false;
		return e;
	}

	LOG(ERROR) << "No encoder found";
	return nullptr;
}

void ftl::codecs::free(Encoder *&enc) {
    UNIQUE_LOCK(mutex, lk);
    enc->reset();
	enc->available = true;
    enc = nullptr;
}

Encoder::Encoder(definition_t maxdef, definition_t mindef, device_t dev) :
		available(true), max_definition(maxdef), min_definition(mindef), device(dev) {

}

Encoder::~Encoder() {

}

bool Encoder::encode(const cv::Mat &in, preset_t preset,
			const std::function<void(const ftl::codecs::Packet&)> &cb) {
	const auto &settings = ftl::codecs::getPreset(preset);
	const definition_t definition = (in.type() == CV_32F) ? settings.depth_res : settings.colour_res;
	const bitrate_t bitrate = (in.type() == CV_32F) ? settings.depth_qual : settings.colour_qual;

	return encode(in, definition, bitrate, cb);
}
