#include <ftl/audio/source.hpp>
#include <ftl/audio/audio.hpp>
#include <ftl/audio/portaudio.hpp>

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

using ftl::audio::Source;
using ftl::audio::Frame;
using ftl::audio::Audio;
using ftl::codecs::Channel;

#ifdef HAVE_PORTAUDIO

//static double ltime = 0.0;

/* Portaudio callback to receive audio data. */
template <typename BUFFER>
static int pa_source_callback(const void *input, void *output,
        unsigned long frameCount, const PaStreamCallbackTimeInfo *timeInfo,
        PaStreamCallbackFlags statusFlags, void *userData) {

    auto *buffer = (BUFFER*)userData;
    float *in = (float*)input;
	buffer->writeFrame(in);
    return 0;
}

#endif

Source::Source(nlohmann::json &config) : ftl::Configurable(config), buffer_(nullptr) {
	if (!value("enabled",true)) {
		active_ = false;
		return;
	}

	#ifdef HAVE_PORTAUDIO
    ftl::audio::pa_init();

	int device = Pa_GetDefaultInputDevice();
	int channels = 1;

	if (get<std::string>("audio_device")) {
		std::string devname = *get<std::string>("audio_device");

        int numDevices = Pa_GetDeviceCount();

        for (int i=0; i<numDevices; ++i) {
            const   PaDeviceInfo *deviceInfo = Pa_GetDeviceInfo(i);
            if (std::string(deviceInfo->name).find(devname) != std::string::npos) {
				device = i;
				break;
			}
        }
	} else {
		device = value("audio_device", device);
		if (device >= Pa_GetDeviceCount()) device = Pa_GetDefaultInputDevice();
	}

	//if (device >= 0) {
		const PaDeviceInfo *deviceInfo = Pa_GetDeviceInfo(device);
		if (deviceInfo) {
			LOG(INFO) << "Using audio device: " << deviceInfo->name;
			if (deviceInfo->maxInputChannels == 0) {
				device = -1;
				LOG(ERROR) << "Selected audio device has no input channels";
			} else {
				channels = (deviceInfo->maxInputChannels >= 2) ? 2 : 1;
			}
		} else {
			LOG(ERROR) << "No selected audio device";
			return;
		}
	//}

	if (channels >= 2) {
		buffer_ = new ftl::audio::StereoBufferF<100>(48000);
	} else {
		buffer_ = new ftl::audio::MonoBufferF<100>(48000);
	}

    PaStreamParameters inputParameters;
    //bzero( &inputParameters, sizeof( inputParameters ) );
    inputParameters.channelCount = channels;
    inputParameters.device = device;
    inputParameters.sampleFormat = paFloat32;
    inputParameters.suggestedLatency = (device >= 0) ? Pa_GetDeviceInfo(device)->defaultLowInputLatency : 0;
    inputParameters.hostApiSpecificStreamInfo = NULL;

	latency_ = int64_t(inputParameters.suggestedLatency * 1000.0);

	PaError err;

	if (inputParameters.device >= 0) { 
		err = Pa_OpenStream(
			&stream_,
			&inputParameters,
			NULL,
			48000,  // Sample rate
			ftl::audio::kFrameSize,    // Size of single frame
			paNoFlag,
			(buffer_->channels() == 1) ? pa_source_callback<ftl::audio::MonoBufferF<100>> : pa_source_callback<ftl::audio::StereoBufferF<100>>,
			this->buffer_
		);
	} else {
		err = Pa_OpenDefaultStream(
			&stream_,
			channels,
			0,
			paFloat32,
			48000,  // Sample rate
			ftl::audio::kFrameSize,    // Size of single frame
			(buffer_->channels() == 1) ? pa_source_callback<ftl::audio::MonoBufferF<100>> : pa_source_callback<ftl::audio::StereoBufferF<100>>,
			this->buffer_
		);
	}

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

	to_read_ = 0;

	settings_.channels = channels;
	settings_.sample_rate = 48000;
	settings_.frame_size = 960;
	//state_.setLeft(settings);

	LOG(INFO) << "Microphone ready.";

	#else  // No portaudio

	active_ = false;
	LOG(ERROR) << "No audio support";

	#endif
}

Source::~Source() {
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

bool Source::capture(int64_t ts) {
	if (buffer_) to_read_ = buffer_->size();
	return true;
}

bool Source::retrieve(ftl::data::Frame &frame) {
	// Remove one interval since the audio starts from the last frame
		//frameset_.timestamp = ts - ftl::timer::getInterval() + latency_;

    if (to_read_ < 1 || !buffer_) return true;
	auto alist = frame.create<std::list<Audio>>((buffer_->channels() == 2) ? Channel::AudioStereo : Channel::AudioMono);
	Audio aframe;
    std::vector<float> &data = aframe.data();
	buffer_->read(data, to_read_);
	alist = std::move(aframe);
	return true;
}
