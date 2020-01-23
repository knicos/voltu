#ifndef _FTL_AUDIO_SPEAKER_HPP_
#define _FTL_AUDIO_SPEAKER_HPP_

#include <ftl/configurable.hpp>
#include <ftl/audio/buffer.hpp>
#include <ftl/audio/frameset.hpp>
#include <ftl/config.h>

#ifdef HAVE_PORTAUDIO
#include <portaudio.h>
#endif

namespace ftl {
namespace audio {

class Speaker : public ftl::Configurable {
	public:
	explicit Speaker(nlohmann::json &config);
	~Speaker();

	void queue(int64_t ts, ftl::audio::Frame &fs);

	void setDelay(int64_t ms);

	private:
	ftl::audio::StereoBuffer16<2000> buffer_;
	bool active_;
	float extra_delay_;

	#ifdef HAVE_PORTAUDIO
	PaStream *stream_;
	#endif
};

}
}

#endif  // _FTL_SPEAKER_HPP_
