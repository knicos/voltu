/**
 * @file speaker.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

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

/**
 * Take data frames, extract the audio data and send to a hardware or other
 * speaker device. Received audio frames are buffered by this class and so this
 * class provides some control of the buffering process also.
 */
class Speaker : public ftl::Configurable {
	public:
	explicit Speaker(nlohmann::json &config);
	~Speaker();

	/** Append new audio frames to internal buffer. */
	void queue(int64_t ts, ftl::audio::Frame &fs);
	void queue(int64_t ts, const ftl::audio::Audio &af);

	void setDelay(int64_t ms);
	void setVolume(float value);
	float volume();

	void reset() { if (buffer_) buffer_->reset(); }

	private:
	ftl::audio::Buffer<float> *buffer_;
	bool active_;
	float extra_delay_;
	float volume_;
	int64_t latency_;

	#ifdef HAVE_PORTAUDIO
	PaStream *stream_;
	#endif

	void _open(int fsize, int sample, int channels);
};

}
}

#endif  // _FTL_SPEAKER_HPP_
