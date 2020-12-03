/**
 * @file frame.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#pragma once
#ifndef _FTL_AUDIO_FRAME_HPP_
#define _FTL_AUDIO_FRAME_HPP_

#include <ftl/data/new_frame.hpp>
#include <ftl/audio/audio.hpp>

namespace ftl {
namespace audio {

static constexpr int kFrameSize = 960;
static constexpr int kSampleRate = 48000;

typedef ftl::data::Frame Frame;
typedef ftl::audio::Audio AudioFrame;

struct AudioSettings {
	int sample_rate;
	int frame_size;
	int channels;
};



}
}

#endif // _FTL_AUDIO_FRAME_HPP_