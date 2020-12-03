/**
 * @file source.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

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

/**
 * Allow audio to act as a general data source for `Frame` objects. Instances
 * of this class can be treated like any other source and will add incoming
 * audio data (from a microphone or other input device) to a `Frame` object
 * at specific timestamps.
 */
class Source : public ftl::Configurable, public ftl::data::DiscreteSource {
    public:
    explicit Source(nlohmann::json &config);
    ~Source();

	/**
	 * Accurately record a timestamp and freeze the data at this time.
	 */
	bool capture(int64_t ts) override;

	/**
	 * Get the already captured audio frames and insert them into the frame
	 * object. Frames arriving after the call to `capture` are not included.
	 */
	bool retrieve(ftl::data::Frame &) override;

    private:
    bool active_;
	ftl::audio::AudioSettings settings_;

	ftl::audio::Buffer<float> *buffer_;
	int to_read_;
	int64_t latency_;

	#ifdef HAVE_PORTAUDIO
	PaStream *stream_;
	#endif
};

}
}

#endif  // _FTL_AUDIO_SOURCE_HPP_
