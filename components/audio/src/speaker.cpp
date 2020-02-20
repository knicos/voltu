#include <ftl/audio/speaker.hpp>
#include <ftl/audio/audio.hpp>
#include <ftl/audio/portaudio.hpp>

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

using ftl::audio::Speaker;
using ftl::audio::Frame;
using ftl::audio::FrameSet;
using ftl::audio::FrameState;
using ftl::audio::Audio;
using ftl::codecs::Channel;

#ifdef HAVE_PORTAUDIO

/* Portaudio callback to receive audio data. */
template <typename BUFFER>
static int pa_speaker_callback(const void *input, void *output,
		unsigned long frameCount, const PaStreamCallbackTimeInfo *timeInfo,
		PaStreamCallbackFlags statusFlags, void *userData) {

	auto *buffer = (BUFFER*)userData;  // ftl::audio::MonoBuffer16<2000>
	short *out = (short*)output;

	buffer->readFrame(out);

	return 0;
}

#endif

Speaker::Speaker(nlohmann::json &config) : ftl::Configurable(config), buffer_(nullptr) {
	#ifdef HAVE_PORTAUDIO
	ftl::audio::pa_init();
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

void Speaker::_open(int fsize, int sample, int channels) {
	if (buffer_) delete buffer_;

	LOG(INFO) << "Create speaker: " << sample << "," << channels;
	if (sample == 0 || channels == 0) return;

	if (channels >= 2) {
		buffer_ = new ftl::audio::StereoBuffer16<2000>(sample);
	} else {
		buffer_ = new ftl::audio::MonoBuffer16<2000>(sample);
	}

	auto err = Pa_OpenDefaultStream(
		&stream_,
		0,
		channels,
		paInt16,
		sample,  // Sample rate
		256,    // Size of single frame
		(channels == 1) ? pa_speaker_callback<ftl::audio::MonoBuffer16<2000>> : pa_speaker_callback<ftl::audio::StereoBuffer16<2000>>,
		this->buffer_
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

	LOG(INFO) << "Speaker ready.";
}

void Speaker::queue(int64_t ts, ftl::audio::Frame &frame) {
	auto &audio = frame.get<ftl::audio::Audio>((frame.hasChannel(Channel::AudioStereo)) ? Channel::AudioStereo : Channel::AudioMono);

	if (!buffer_) {
		_open(256, frame.getSettings().sample_rate, frame.getSettings().channels);
	}
	if (!buffer_) return;

	//LOG(INFO) << "Buffer Fullness (" << ts << "): " << buffer_->size() << " - " << audio.size();
	buffer_->write(audio.data());
	//LOG(INFO) << "Audio delay: " << buffer_.delay() << "s";
}

void Speaker::setDelay(int64_t ms) {
	float d = static_cast<float>(ms) / 1000.0f + extra_delay_;
	if (d < 0.0f) d = 0.0f;  // Clamp to 0 delay (not ideal to be exactly 0)
	if (buffer_) buffer_->setDelay(d);
}
