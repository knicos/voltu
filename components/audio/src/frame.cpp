/**
 * @file frame.cpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#include <ftl/audio/frame.hpp>
#include <ftl/audio/audio.hpp>

using ftl::audio::Audio;
using ftl::audio::AudioData;

template <> Audio &AudioData::as<Audio>() {
	return data;
}

template <> const Audio &AudioData::as<Audio>() const {
	return data;
}

template <> Audio &AudioData::make<Audio>() {
	return data;
}
