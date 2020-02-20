#pragma once
#ifndef _FTL_AUDIO_FRAME_HPP_
#define _FTL_AUDIO_FRAME_HPP_

#include <ftl/data/framestate.hpp>
#include <ftl/data/frame.hpp>
#include <ftl/audio/audio.hpp>

namespace ftl {
namespace audio {

struct AudioSettings {
	int sample_rate;
	int frame_size;
	int channels;
};

struct AudioData {
	template <typename T>
	const T &as() const {
		throw FTL_Error("Type not valid for audio channel");
	}

	template <typename T>
	T &as() {
		throw FTL_Error("Type not valid for audio channel");
	}

	template <typename T>
	T &make() {
		throw FTL_Error("Type not valid for audio channel");
	}

	inline void reset() {}

	Audio data;
};

// Specialisations for getting Audio data.
template <> Audio &AudioData::as<Audio>(); 
template <> const Audio &AudioData::as<Audio>() const;
template <> Audio &AudioData::make<Audio>();

typedef ftl::data::FrameState<AudioSettings,2> FrameState;
typedef ftl::data::Frame<32,2,FrameState,AudioData> Frame;

}
}

#endif // _FTL_AUDIO_FRAME_HPP_