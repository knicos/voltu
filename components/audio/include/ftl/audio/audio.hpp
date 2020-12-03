/**
 * @file audio.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#ifndef _FTL_AUDIO_AUDIO_HPP_
#define _FTL_AUDIO_AUDIO_HPP_

#include <vector>

namespace ftl {
namespace audio {

/**
 * Raw storage for one or more audio frames. Used within a `Frame` channel.
 * The data here is already decoded / uncompressed.
 */
class Audio {
	public:
	Audio() {};

	size_t size() const { return data_.size()*sizeof(short); }

	std::vector<float> &data() { return data_; }
	const std::vector<float> &data() const { return data_; }

	private:
	std::vector<float> data_;
};

}
}

/* Helper for the `Frame` class */
template <>
inline bool ftl::data::make_type<std::list<ftl::audio::Audio>>() {
	return false;
}

/* Prevent raw decode */
template <>
inline bool ftl::data::decode_type<std::list<ftl::audio::Audio>>(std::any &a, const std::vector<uint8_t> &data) {
	return false;
}

#endif  // _FTL_AUDIO_AUDIO_HPP_
