/**
 * @file portaudio.cpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#include <ftl/audio/portaudio.hpp>
#include <ftl/config.h>
#include <ftl/threads.hpp>
#include <loguru.hpp>

#include <atomic>

static std::atomic<int> counter = 0;
static MUTEX pa_mutex;

#ifdef HAVE_PORTAUDIO

#include <portaudio.h>

void ftl::audio::pa_init() {
    UNIQUE_LOCK(pa_mutex, lk);
    if (counter == 0) {
        auto err = Pa_Initialize();
        if (err != paNoError) {
            LOG(ERROR) << "Portaudio failed to initialise: " << Pa_GetErrorText(err);
            counter = 1000;
        }

        // List devices
        int numDevices = Pa_GetDeviceCount();

        if (numDevices == 0) LOG(WARNING) << "No audio devices found";
        else LOG(INFO) << "Audio devices:";

        for (int i=0; i<numDevices; ++i) {
            const   PaDeviceInfo *deviceInfo = Pa_GetDeviceInfo(i);
            LOG(INFO) << " -- (" << i << ") " << deviceInfo->name << " - Inputs=" << deviceInfo->maxInputChannels << " Outputs=" << deviceInfo->maxOutputChannels;
        }
    }
    ++counter;
}

void ftl::audio::pa_final() {
    UNIQUE_LOCK(pa_mutex, lk);
    --counter;
    if (counter == 0) {
        auto err = Pa_Terminate();
        if (err != paNoError) {
            LOG(ERROR) << "Portaudio failed to terminate: " << Pa_GetErrorText(err);
            counter = -1000;
        }
    }
}
#endif