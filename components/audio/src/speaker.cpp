#include <ftl/audio/speaker.hpp>
#include <ftl/audio/audio.hpp>
#include <ftl/audio/portaudio.hpp>

using ftl::audio::Speaker;
using ftl::audio::Frame;
using ftl::audio::FrameSet;
using ftl::audio::FrameState;
using ftl::audio::Audio;
using ftl::codecs::Channel;

#ifdef HAVE_PORTAUDIO

/* Portaudio callback to receive audio data. */
static int pa_speaker_callback(const void *input, void *output,
		unsigned long frameCount, const PaStreamCallbackTimeInfo *timeInfo,
		PaStreamCallbackFlags statusFlags, void *userData) {

	auto *buffer = (ftl::audio::StereoBuffer16<2000>*)userData;
	short *out = (short*)output;

	buffer->readFrame(out);

	return 0;
}

#endif

Speaker::Speaker(nlohmann::json &config) : ftl::Configurable(config), buffer_(48000) {
	#ifdef HAVE_PORTAUDIO
	ftl::audio::pa_init();

	auto err = Pa_OpenDefaultStream(
		&stream_,
		0,
		2,
		paInt16,
		48000,  // Sample rate
		256,    // Size of single frame
		pa_speaker_callback,
		&this->buffer_
	);

	if (err != paNoError) {
		LOG(ERROR) << "Portaudio open stream error: " << Pa_GetErrorText(err);
		active_ = false;
		return;
	} else {
		active_ = true;
	}

	err = Pa_StartStream(stream_);

	if (err != paNoError) {
		LOG(ERROR) << "Portaudio start stream error: " << Pa_GetErrorText(err);
		//active_ = false;
		return;
	}

	#else  // No portaudio

	active_ = false;
	LOG(ERROR) << "No audio support";

	#endif

	extra_delay_ = value("delay",0.0f);
	on("delay", [this](const ftl::config::Event &e) {
		extra_delay_ = value("delay",0.0f);
	});
}

Speaker::~Speaker() {
	if (active_) {
		active_ = false;

		#ifdef HAVE_PORTAUDIO
		auto err = Pa_StopStream(stream_);

		if (err != paNoError) {
			LOG(ERROR) << "Portaudio stop stream error: " << Pa_GetErrorText(err);
			//active_ = false;
		}

		err = Pa_CloseStream(stream_);

		if (err != paNoError) {
			LOG(ERROR) << "Portaudio close stream error: " << Pa_GetErrorText(err);
		}
		#endif
	}

	#ifdef HAVE_PORTAUDIO
	ftl::audio::pa_final();
	#endif
}

void Speaker::queue(int64_t ts, ftl::audio::Frame &frame) {
	auto &audio = frame.get<ftl::audio::Audio>(Channel::Audio);

	//LOG(INFO) << "Buffer Fullness (" << ts << "): " << buffer_.size();
	buffer_.write(audio.data());
	//LOG(INFO) << "Audio delay: " << buffer_.delay() << "s";
}

void Speaker::setDelay(int64_t ms) {
	float d = static_cast<float>(ms) / 1000.0f + extra_delay_;
	if (d < 0.0f) d = 0.0f;  // Clamp to 0 delay (not ideal to be exactly 0)
	buffer_.setDelay(d);
}
