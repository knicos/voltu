/**
 * @file speaker.cpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#include <ftl/audio/speaker.hpp>
#include <ftl/audio/audio.hpp>
#include <ftl/audio/portaudio.hpp>

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

using ftl::audio::Speaker;
using ftl::audio::Frame;
using ftl::audio::FrameSet;
using ftl::audio::Audio;
using ftl::codecs::Channel;

#ifdef HAVE_PORTAUDIO

/* Portaudio callback to receive audio data. */
template <typename BUFFER>
static int pa_speaker_callback(const void *input, void *output,
		unsigned long frameCount, const PaStreamCallbackTimeInfo *timeInfo,
		PaStreamCallbackFlags statusFlags, void *userData) {

	auto *buffer = (BUFFER*)userData;  // ftl::audio::MonoBuffer16<2000>
	float *out = (float*)output;

	buffer->readFrame(out);

	return 0;
}

#endif

Speaker::Speaker(nlohmann::json &config) : ftl::Configurable(config), buffer_(nullptr), stream_(nullptr) {
	#ifdef HAVE_PORTAUDIO
	ftl::audio::pa_init();
	#else  // No portaudio
	LOG(ERROR) << "No audio support";
	#endif
	volume_ = 1.0f;
	active_ = false;
	extra_delay_ = value("delay",0.1f);
	on("delay", [this]() {
		extra_delay_ = value("delay",0.1f);
		setDelay(0);
	});
	setDelay(0);
}

Speaker::~Speaker() {
	if (active_) {
		active_ = false;

		#ifdef HAVE_PORTAUDIO
		auto err = Pa_AbortStream(stream_);

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
	#ifdef HAVE_PORTAUDIO

	if (buffer_) delete buffer_;

	LOG(INFO) << "Create speaker: " << sample << "," << channels;
	if (sample == 0 || channels == 0) return;

	if (channels >= 2) {
		buffer_ = new ftl::audio::StereoBufferF<2000>(sample);
	} else {
		buffer_ = new ftl::audio::MonoBufferF<2000>(sample);
	}

	PaStreamParameters outputParameters;
	//bzero( &inputParameters, sizeof( inputParameters ) );
	outputParameters.channelCount = channels;
	outputParameters.device = Pa_GetDefaultOutputDevice();
	outputParameters.sampleFormat = paFloat32;
	outputParameters.suggestedLatency = Pa_GetDeviceInfo(outputParameters.device)->defaultLowOutputLatency;
	outputParameters.hostApiSpecificStreamInfo = NULL;

	latency_ = int64_t(outputParameters.suggestedLatency * 1000.0);

	auto err = Pa_OpenStream(
		&stream_,
		NULL,
		&outputParameters,
		sample,	// Sample rate
		960,	// Size of single frame
		paNoFlag,
		(channels == 1) ? pa_speaker_callback<ftl::audio::MonoBufferF<2000>> : pa_speaker_callback<ftl::audio::StereoBufferF<2000>>,
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
	#else
	LOG(INFO) << "Built without portaudio (no sound)";
	#endif
}

void Speaker::queue(int64_t ts, ftl::audio::Frame &frame) {
	const auto &audio = frame.get<std::list<ftl::audio::Audio>>
		((frame.hasChannel(Channel::AudioStereo)) ? Channel::AudioStereo : Channel::AudioMono);

	if (!buffer_) {
		_open(960, 48000, (frame.hasChannel(Channel::AudioStereo)) ? 2 : 1);
	}
	if (!buffer_) return;

	for (const auto &d : audio) {
		buffer_->write(d.data());
	}
}

void Speaker::queue(int64_t ts, const ftl::audio::Audio &d) {
	if (!buffer_) {
		_open(960, 48000, 2);
	}
	if (!buffer_) return;

	buffer_->write(d.data());
}

void Speaker::setDelay(int64_t ms) {
	ms -= latency_;
	float d = static_cast<float>(ms) / 1000.0f + extra_delay_;
	if (d < 0.0f) d = 0.001f;  // Clamp to 0 delay (not ideal to be exactly 0)
	if (buffer_) {
		buffer_->setDelay(d);
		LOG(INFO) << "Audio delay: " << buffer_->delay();
	}
}

void Speaker::setVolume(float value) {
	// TODO: adjust volume using system mixer
	volume_ = std::max(0.0f, std::min(1.0f, value));
	if (buffer_) buffer_->setGain(volume_);
}

float Speaker::volume() {
	return volume_;
}
