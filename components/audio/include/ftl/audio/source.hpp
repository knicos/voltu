#ifndef _FTL_AUDIO_SOURCE_HPP_
#define _FTL_AUDIO_SOURCE_HPP_

#include <ftl/audio/buffer.hpp>
#include <ftl/audio/frameset.hpp>
#include <ftl/data/creators.hpp>
#include <ftl/configurable.hpp>
#include <ftl/config.h>

#ifdef HAVE_PORTAUDIO
#include <portaudio.h>
#endif

namespace ftl {
namespace audio {

class Source : public ftl::Configurable, public ftl::data::DiscreteSource {
    public:
    explicit Source(nlohmann::json &config);
    ~Source();

	bool capture(int64_t ts) override;

	bool retrieve(ftl::data::Frame &) override;

    private:
    bool active_;
	ftl::audio::AudioSettings settings_;

	ftl::audio::Buffer<short> *buffer_;
	int to_read_;
	int64_t latency_;

	#ifdef HAVE_PORTAUDIO
	PaStream *stream_;
	#endif
};

}
}

#endif  // _FTL_AUDIO_SOURCE_HPP_
