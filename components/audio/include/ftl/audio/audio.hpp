#ifndef _FTL_AUDIO_AUDIO_HPP_
#define _FTL_AUDIO_AUDIO_HPP_

#include <vector>

namespace ftl {
namespace audio {

class Audio {
	public:
	Audio() {};

	size_t size() const { return data_.size()*sizeof(short); }

	std::vector<short> &data() { return data_; }
	const std::vector<short> &data() const { return data_; }

	private:
	std::vector<short> data_;
};

}
}

template <>
inline bool ftl::data::make_type<std::list<ftl::audio::Audio>>() {
	return false;
}

template <>
inline bool ftl::data::decode_type<std::list<ftl::audio::Audio>>(std::any &a, const std::vector<uint8_t> &data) {
	return false;
}

#endif  // _FTL_AUDIO_AUDIO_HPP_
