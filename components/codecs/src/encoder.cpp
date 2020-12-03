/**
 * @file encoder.cpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#include <ftl/codecs/encoder.hpp>
#include <ftl/threads.hpp>

#include <list>

#include <ftl/config.h>
#include <loguru.hpp>

using ftl::codecs::Encoder;
using ftl::codecs::device_t;
using ftl::codecs::codec_t;

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

Encoder *ftl::codecs::allocateEncoder(ftl::codecs::device_t dev, ftl::codecs::codec_t codec) {
	UNIQUE_LOCK(mutex, lk);
	if (!has_been_init) init_encoders();

	for (auto i=encoders.begin(); i!=encoders.end(); ++i) {
		auto *e = *i;
		if (!e->available) continue;
		if (dev != device_t::Any && dev != e->device_) continue;
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

Encoder::Encoder(device_t dev) :
		available(true), device_(dev) {

}

Encoder::~Encoder() {

}

std::pair<int,int> ftl::codecs::chooseTileConfig(int size) {
	switch (size) {
	case 1:		return {1,1};
	case 2:		return {2,1};
	case 3:		return {2,2};
	case 4:		return {2,2};
	case 5:		return {3,2};
	case 6:		return {3,2};
	case 7:		return {4,2};
	case 8:		return {4,2};
	case 9:		return {3,3};
	}
	return {3,3};
}
