#include <ftl/audio/source.hpp>
#include <ftl/audio/audio.hpp>
#include <ftl/audio/portaudio.hpp>

using ftl::audio::Source;
using ftl::audio::Frame;
using ftl::audio::FrameSet;
using ftl::audio::FrameState;
using ftl::audio::Audio;
using ftl::codecs::Channel;

#ifdef HAVE_PORTAUDIO

//static double ltime = 0.0;

/* Portaudio callback to receive audio data. */
static int pa_source_callback(const void *input, void *output,
        unsigned long frameCount, const PaStreamCallbackTimeInfo *timeInfo,
        PaStreamCallbackFlags statusFlags, void *userData) {

    auto *buffer = (ftl::audio::StereoBuffer16<100>*)userData;
    short *in = (short*)input;

	//short *out = (short*)output;
	//buffer->readFrame(out);

	//if (timeInfo->currentTime - ltime < (1.0 / 128.0)) return 0;
	//ltime = timeInfo->inputBufferAdcTime;

    //int i=0;
    //while (i < frameCount) {
	    buffer->writeFrame(in);
        //i+=2*ftl::audio::kFrameSize;
    //

    return 0;
}

#endif

Source::Source(nlohmann::json &config) : ftl::Configurable(config), buffer_(48000) {
	if (!value("enabled",true)) {
		active_ = false;
		return;
	}

	#ifdef HAVE_PORTAUDIO
    ftl::audio::pa_init();

	int device = value("audio_device",-1);
	if (device >= Pa_GetDeviceCount()) device = -1;

    PaStreamParameters inputParameters;
    //bzero( &inputParameters, sizeof( inputParameters ) );
    inputParameters.channelCount = 2;
    inputParameters.device = device;
    inputParameters.sampleFormat = paInt16;
    inputParameters.suggestedLatency = (device >= 0) ? Pa_GetDeviceInfo(device)->defaultLowInputLatency : 0;
    inputParameters.hostApiSpecificStreamInfo = NULL;

	PaError err;

	if (inputParameters.device >= 0) { 
		err = Pa_OpenStream(
			&stream_,
			&inputParameters,
			NULL,
			48000,  // Sample rate
			ftl::audio::kFrameSize,    // Size of single frame
			paNoFlag,
			pa_source_callback,
			&this->buffer_
		);
	} else {
		err = Pa_OpenDefaultStream(
			&stream_,
			2,
			0,
			paInt16,
			48000,  // Sample rate
			ftl::audio::kFrameSize,    // Size of single frame
			pa_source_callback,
			&this->buffer_
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

    timer_hp_ = ftl::timer::add(ftl::timer::kTimerHighPrecision, [this](int64_t ts) {
        to_read_ = buffer_.size();
        return true;
    });

	timer_main_ = ftl::timer::add(ftl::timer::kTimerMain, [this](int64_t ts) {

        // Remove one interval since the audio starts from the last frame
		frameset_.timestamp = ts - ftl::timer::getInterval();

		frameset_.id = 0;
		frameset_.count = 1;
		frameset_.stale = false;

        if (to_read_ < 1) return true;

		if (frameset_.frames.size() < 1) frameset_.frames.emplace_back();

		auto &frame = frameset_.frames[0];
		frame.reset();
        std::vector<short> &data = frame.create<Audio>(Channel::Audio).data();

		data.resize(2*ftl::audio::kFrameSize*to_read_);
		short *ptr = data.data();
		for (int i=0; i<to_read_; ++i) {
			buffer_.readFrame(ptr);
			ptr += 2*ftl::audio::kFrameSize;
		}

		// Then do something with the data!
		//LOG(INFO) << "Audio Frames Sent: " << to_read_ << " - " << ltime;
		if (cb_) cb_(frameset_);

        return true;
    }); 

	#else  // No portaudio

	active_ = false;
	LOG(ERROR) << "No audio support";

	#endif
}

Source::~Source() {
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

size_t Source::size() {
    return 1;
}

ftl::audio::FrameState &Source::state(int ix) {
    if (ix < 0 || ix > 1) throw ftl::exception("State index out-of-bounds");
    return state_;
}

void Source::onFrameSet(const ftl::audio::FrameSet::Callback &cb) {
	cb_ = cb;
}
